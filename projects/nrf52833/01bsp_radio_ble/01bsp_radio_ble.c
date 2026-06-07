/**
\brief This program shows the use of the "radio" bsp module.

Since the bsp modules for different platforms have the same declaration, you
can use this project with any platform.

After loading this program, your board will switch on its radio on frequency
CHANNEL.

While receiving a packet (i.e. from the start of frame event to the end of
frame event), it will turn on its sync LED.

Every TIMER_PERIOD, it will also send a packet containing LENGTH_PACKET bytes
set to ID. While sending a packet (i.e. from the start of frame event to the
end of frame event), it will turn on its error LED.

\author Tengfei Chang <tengfei.chang@inria.fr>, August 2020.
*/

#include "board.h"
#include "nrf52833.h"
#include "nrf52833_bitfields.h"
#include "radio.h"
#include "leds.h"
#include "sctimer.h"
#include "uart.h"
#include "i2c.h"
#include "radio_df.h"
#include "bmi270.h"

//=========================== defines =========================================

#define LENGTH_PACKET   37             ///< S0 + length + BLE advertising payload
#define SCTIMER_TICKS_PER_SECOND 32768UL
#define SCTIMER_TIMERMASK        0x00ffffffUL
#define ADV_INTERVAL_MAX_S       511UL
#define BLE_TX_LED_ON_TICKS      (SCTIMER_TICKS_PER_SECOND/2)
#define BLE_IDLE_LED_ON_TICKS    (SCTIMER_TICKS_PER_SECOND/10)
#define BLE_IDLE_LED_OFF_TICKS   SCTIMER_TICKS_PER_SECOND

#define NUM_SAMPLES     SAMPLE_MAXCNT
#define LEN_UART_BUFFER ((NUM_SAMPLES*4)+8)

#define ENABLE_DF       0

#define BMI270_DIAG_PRESENT       0x01
#define BMI270_DIAG_READ_OK       0x02
#define BMI270_DIAG_CHIPID_OK     0x04
#define BMI270_DIAG_ALT_ADDR      0x08
#define BMI270_DIAG_INIT_OK       0x10

#define BLE_ADV_LED_PIN           9

const static uint8_t ble_device_addr[6] = { 
    0xaa, 0xbb, 0xcc, 0xcc, 0xbb, 0xea
};

const static uint8_t ble_adv_channels[] = {
    37
};

const static uint8_t ble_device_name[] = {
    'n', 'o', 'r', 'd', 'i', 'c'
};

volatile bool     g_bmi271_enabled = TRUE;
volatile bool     g_led_enabled = TRUE;
volatile uint32_t g_adv_interval_s = 1;
volatile uint32_t g_startup_sleep_s = 1;

//=========================== variables =======================================

enum {
    APP_FLAG_END_FRAME   = 0x01,
    APP_FLAG_TIMER       = 0x02,
};

enum {
    APP_TIMER_ADV        = 0x00,
    APP_TIMER_LED_ON     = 0x01,
    APP_TIMER_LED_OFF    = 0x02,
};

typedef enum {
    APP_STATE_TX         = 0x01,
    APP_STATE_RX         = 0x02,
} app_state_t;

typedef struct {
    uint8_t              num_startFrame;
    uint8_t              num_endFrame;
    uint8_t              num_timer;
} app_dbg_t;

app_dbg_t app_dbg;

typedef struct {
     volatile   uint8_t         flags;
                app_state_t     state;
                uint8_t         packet[LENGTH_PACKET];
                uint8_t         packet_len;
                int8_t          rxpk_rssi;
                uint8_t         rxpk_lqi;
                bool            rxpk_crc;
                uint16_t        num_samples;
                uint8_t         adv_channel_index;
                bool            bmi270_present;
                bool            bmi270_read_ok;
                bool            led_on;
                uint8_t         timer_event;
                uint8_t         bmi270_addr;
                uint8_t         bmi270_diag;
                uint8_t         bmi270_who_am_i;
                uint8_t         bmi270_status;
                uint8_t         bmi270_error_reg;
                uint8_t         bmi270_internal_status;
                int16_t         acc_x;
                int16_t         acc_y;
                int16_t         acc_z;
                PORT_TIMER_WIDTH next_adv_time;
                PORT_TIMER_WIDTH next_led_time;
                PORT_TIMER_WIDTH led_off_time;
                uint32_t        sample_buffer[NUM_SAMPLES];
                uint8_t         uart_buffer_to_send[LEN_UART_BUFFER];
                uint16_t        uart_lastTxByteIndex;
     volatile   uint8_t         uartDone;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);
void     cb_timer(void);

void     cb_uartTxDone(void);
uint8_t  cb_uartRxCb(void);

void     assemble_adv_name_packet(void);
PORT_TIMER_WIDTH get_adv_period_ticks(void);
PORT_TIMER_WIDTH get_startup_sleep_ticks(void);
void     startup_lowpower_sleep(void);
void     init_bmi270(void);
void     update_bmi270_sample(void);
void     send_next_adv_packet(void);

//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {
    uint16_t i;
    PORT_TIMER_WIDTH now;

    // clear local variables
    memset(&app_vars,0,sizeof(app_vars_t));

    // keep only the RTC running during the startup low-power sleep
    sctimer_init();
    sctimer_set_callback(cb_timer);
    sctimer_enable();
    startup_lowpower_sleep();

    // initialize board after the startup low-power sleep
    board_init();

    // board_init reinitializes the timer, so restore the app callback
    sctimer_set_callback(cb_timer);
    sctimer_enable();

#if ENABLE_DF == 1
    radio_configure_direction_finding_antenna_switch();

#endif
    // uart_setCallbacks(cb_uartTxDone,cb_uartRxCb);
    // uart_enableInterrupts();

    init_bmi270();

    // add callback functions radio
    radio_setEndFrameCb(cb_endFrame);

#if ENABLE_DF == 1
    radio_configure_direction_finding_manual();
#endif

    // stay idle between advertising packets
    app_vars.state = APP_STATE_RX;

    // start by a transmit after the startup low-power sleep
    app_vars.flags |= APP_FLAG_TIMER;

    while (1) {

        // sleep while waiting for at least one of the flags to be set
        while (app_vars.flags==0x00) {
            board_sleep();
        }

        // handle and clear every flag
        while (app_vars.flags) {

            //==== APP_FLAG_END_FRAME (TX or RX)

            if (app_vars.flags & APP_FLAG_END_FRAME) {
                // end of frame

                switch (app_vars.state) {

                    case APP_STATE_RX:

                        // done receiving a packet
                        app_vars.packet_len = sizeof(app_vars.packet);

                        // get packet from radio
                        radio_getReceivedFrame(
                            app_vars.packet,
                            &app_vars.packet_len,
                            sizeof(app_vars.packet),
                            &app_vars.rxpk_rssi,
                            &app_vars.rxpk_lqi,
                            &app_vars.rxpk_crc
                        );
                        if (app_vars.rxpk_crc && ENABLE_DF) {
                            
                            app_vars.num_samples = radio_get_df_samples(app_vars.sample_buffer,NUM_SAMPLES);

                            // record the samples
                            for (i=0;i<app_vars.num_samples;i++) {
                                app_vars.uart_buffer_to_send[4*i+0] = (app_vars.sample_buffer[i] >>24) & 0x000000ff;
                                app_vars.uart_buffer_to_send[4*i+1] = (app_vars.sample_buffer[i] >>16) & 0x000000ff;
                                app_vars.uart_buffer_to_send[4*i+2] = (app_vars.sample_buffer[i] >> 8) & 0x000000ff;
                                app_vars.uart_buffer_to_send[4*i+3] = (app_vars.sample_buffer[i] >> 0) & 0x000000ff;
                            }

                            // recoard rssi
                            app_vars.uart_buffer_to_send[4*i+0]     = app_vars.rxpk_rssi;
                            // record scum settings for transmitting
                            app_vars.uart_buffer_to_send[4*i+1]     = app_vars.packet[3];
                            app_vars.uart_buffer_to_send[4*i+2]     = app_vars.packet[4];
                            app_vars.uart_buffer_to_send[4*i+3]     = app_vars.packet[5];
                            // frame split identifier
                            app_vars.uart_buffer_to_send[4*i+4]     = 0xff;
                            app_vars.uart_buffer_to_send[4*i+5]     = 0xff;
                            app_vars.uart_buffer_to_send[4*i+6]     = 0xff;
                            app_vars.uart_buffer_to_send[4*i+7]     = 0xff;

                            app_vars.uart_lastTxByteIndex = 0;
                            uart_writeByte(app_vars.uart_buffer_to_send[0]);
                        }

                        break;
                    case APP_STATE_TX:
                        // done sending a packet

                        if (g_bmi271_enabled==TRUE && app_vars.bmi270_present==TRUE) {
                            bmi270_power_down();
                        }
                        radio_rfOff();
                        i2c_disable();
                        if (g_led_enabled==TRUE) {
                            NRF_P1->OUTCLR = (uint32_t)1 << BLE_ADV_LED_PIN;
                            NRF_P1->PIN_CNF[BLE_ADV_LED_PIN] =
                                  ((uint32_t)GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
                                | ((uint32_t)GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                                | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
                        }

                        // sleep until the next advertising event
                        app_vars.state = APP_STATE_RX;
                        app_vars.adv_channel_index = 0;
                        sctimer_setCompare(sctimer_readCounter()+get_adv_period_ticks());
                        break;
                }
                // clear flag
                app_vars.flags &= ~APP_FLAG_END_FRAME;
            }

            //==== APP_FLAG_TIMER

            if (app_vars.flags & APP_FLAG_TIMER) {
                // timer fired

                if ((app_vars.timer_event==APP_TIMER_LED_OFF) && (app_vars.led_on==TRUE)) {
                    now = sctimer_readCounter();
                    NRF_P1->OUTCLR = (uint32_t)1 << BLE_ADV_LED_PIN;
                    NRF_P1->PIN_CNF[BLE_ADV_LED_PIN] =
                          ((uint32_t)GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos)
                        | ((uint32_t)GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                        | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                        | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                        | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
                    app_vars.led_on = FALSE;
                    app_vars.next_led_time = now+BLE_IDLE_LED_OFF_TICKS;
                    if (((app_vars.next_adv_time-app_vars.next_led_time) & SCTIMER_TIMERMASK) < (SCTIMER_TIMERMASK/2)) {
                        app_vars.timer_event = APP_TIMER_LED_ON;
                        sctimer_setCompare(app_vars.next_led_time);
                    } else {
                        app_vars.timer_event = APP_TIMER_ADV;
                        sctimer_setCompare(app_vars.next_adv_time);
                    }
                } else if ((app_vars.timer_event==APP_TIMER_LED_ON) && (app_vars.state==APP_STATE_RX)) {
                    app_vars.led_on = TRUE;
                    app_vars.led_off_time = sctimer_readCounter()+BLE_IDLE_LED_ON_TICKS;
                    NRF_P1->OUTCLR = (uint32_t)1 << BLE_ADV_LED_PIN;
                    NRF_P1->PIN_CNF[BLE_ADV_LED_PIN] =
                          ((uint32_t)GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos)
                        | ((uint32_t)GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
                        | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                        | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                        | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
                    NRF_P1->OUTSET = (uint32_t)1 << BLE_ADV_LED_PIN;
                    if (((app_vars.next_adv_time-app_vars.led_off_time) & SCTIMER_TIMERMASK) < (SCTIMER_TIMERMASK/2)) {
                        app_vars.timer_event = APP_TIMER_LED_OFF;
                        sctimer_setCompare(app_vars.led_off_time);
                    } else {
                        app_vars.timer_event = APP_TIMER_ADV;
                        sctimer_setCompare(app_vars.next_adv_time);
                    }
                } else if (app_vars.state==APP_STATE_RX) {
                    app_vars.adv_channel_index = 0;
                    send_next_adv_packet();
                }

                // clear flag
                app_vars.flags &= ~APP_FLAG_TIMER;
            }
        }
    }
}
//=========================== private =========================================

void assemble_adv_name_packet(void) {

    uint8_t  i;
    uint8_t  j;
    uint16_t acc_x;
    uint16_t acc_y;
    uint16_t acc_z;
    i=0;
    acc_x = (uint16_t)app_vars.acc_x;
    acc_y = (uint16_t)app_vars.acc_y;
    acc_z = (uint16_t)app_vars.acc_z;

    memset( app_vars.packet, 0x00, sizeof(app_vars.packet) );

    app_vars.packet[i++]  = 0x40;               // BLE ADV_IND, random AdvA
    app_vars.packet[i++]  = 0x1f;               // Payload length
    app_vars.packet[i++]  = ble_device_addr[0]; // BLE adv address byte 0
    app_vars.packet[i++]  = ble_device_addr[1]; // BLE adv address byte 1
    app_vars.packet[i++]  = ble_device_addr[2]; // BLE adv address byte 2
    app_vars.packet[i++]  = ble_device_addr[3]; // BLE adv address byte 3
    app_vars.packet[i++]  = ble_device_addr[4]; // BLE adv address byte 4
    app_vars.packet[i++]  = ble_device_addr[5]; // BLE adv address byte 5

    app_vars.packet[i++]  = 0x02;               // Flags AD length
    app_vars.packet[i++]  = 0x01;               // Flags AD type
    app_vars.packet[i++]  = 0x06;               // LE General Discoverable, BR/EDR not supported

    app_vars.packet[i++]  = sizeof(ble_device_name) + 1;
    app_vars.packet[i++]  = 0x09;               // Complete Local Name AD type
    for (j=0;j<sizeof(ble_device_name);j++) {
        app_vars.packet[i++] = ble_device_name[j];
    }

    app_vars.packet[i++]  = 0x0d;               // Manufacturer AD length
    app_vars.packet[i++]  = 0xff;               // Manufacturer Specific Data type
    app_vars.packet[i++]  = 0xff;               // Company ID LSB, test value
    app_vars.packet[i++]  = 0xff;               // Company ID MSB, test value
    if (g_bmi271_enabled==TRUE) {
        app_vars.packet[i++]  = app_vars.bmi270_who_am_i;
        app_vars.packet[i++]  = app_vars.bmi270_diag;
        app_vars.packet[i++]  = app_vars.bmi270_status;
        app_vars.packet[i++]  = app_vars.bmi270_internal_status;
        app_vars.packet[i++]  = (uint8_t)((acc_x >> 0) & 0x00ff);
        app_vars.packet[i++]  = (uint8_t)((acc_x >> 8) & 0x00ff);
        app_vars.packet[i++]  = (uint8_t)((acc_y >> 0) & 0x00ff);
        app_vars.packet[i++]  = (uint8_t)((acc_y >> 8) & 0x00ff);
        app_vars.packet[i++]  = (uint8_t)((acc_z >> 0) & 0x00ff);
        app_vars.packet[i++]  = (uint8_t)((acc_z >> 8) & 0x00ff);
    } else {
        for (j=0;j<10;j++) {
            app_vars.packet[i++] = 0xaa;
        }
    }

    app_vars.packet_len   = i;
}

PORT_TIMER_WIDTH get_adv_period_ticks(void) {
    uint32_t interval_s;

    interval_s = g_adv_interval_s;
    if (interval_s==0) {
        interval_s = 1;
    }
    if (interval_s>ADV_INTERVAL_MAX_S) {
        interval_s = ADV_INTERVAL_MAX_S;
    }

    return (PORT_TIMER_WIDTH)(interval_s*SCTIMER_TICKS_PER_SECOND);
}

PORT_TIMER_WIDTH get_startup_sleep_ticks(void) {
    uint32_t startup_sleep_s;

    startup_sleep_s = g_startup_sleep_s;
    if (startup_sleep_s>ADV_INTERVAL_MAX_S) {
        startup_sleep_s = ADV_INTERVAL_MAX_S;
    }

    return (PORT_TIMER_WIDTH)(startup_sleep_s*SCTIMER_TICKS_PER_SECOND);
}

void startup_lowpower_sleep(void) {

    if (g_startup_sleep_s==0) {
        return;
    }

    app_vars.flags &= ~APP_FLAG_TIMER;
    sctimer_setCompare(sctimer_readCounter()+get_startup_sleep_ticks());

    while ((app_vars.flags & APP_FLAG_TIMER)==0) {
        board_sleep();
    }
    app_vars.flags &= ~APP_FLAG_TIMER;

}

void init_bmi270(void) {

    if (g_bmi271_enabled==FALSE) {
        i2c_init();
        i2c_set_addr(BMI270_ADDR);
        bmi270_power_down();
        i2c_set_addr(BMI270_ADDR_ALT);
        bmi270_power_down();
        i2c_disable();
        app_vars.bmi270_present = FALSE;
        app_vars.bmi270_read_ok = FALSE;
        app_vars.bmi270_diag = 0;
        app_vars.bmi270_who_am_i = 0;
        app_vars.bmi270_status = 0;
        app_vars.bmi270_error_reg = 0;
        app_vars.bmi270_internal_status = 0;
        app_vars.acc_x = 0;
        app_vars.acc_y = 0;
        app_vars.acc_z = 0;
        return;
    }

    i2c_init();

    app_vars.bmi270_addr = BMI270_ADDR;
    i2c_set_addr(app_vars.bmi270_addr);
    app_vars.bmi270_who_am_i = bmi270_who_am_i();
    app_vars.bmi270_read_ok = (bmi270_last_i2c_result()!=0);

    if (app_vars.bmi270_who_am_i!=BMI270_CHIPID) {
        app_vars.bmi270_addr = BMI270_ADDR_ALT;
        i2c_set_addr(app_vars.bmi270_addr);
        app_vars.bmi270_who_am_i = bmi270_who_am_i();
        app_vars.bmi270_read_ok = (bmi270_last_i2c_result()!=0);
    }

    if (app_vars.bmi270_who_am_i==BMI270_CHIPID) {
        app_vars.bmi270_present = (bmi270_default_config()!=0);
        app_vars.bmi270_status = bmi270_get_status();
        app_vars.bmi270_error_reg = bmi270_get_errorreg();
        app_vars.bmi270_internal_status = bmi270_get_internal_status();
    }

    app_vars.bmi270_diag = 0;
    if (app_vars.bmi270_present==TRUE) {
        app_vars.bmi270_diag |= BMI270_DIAG_PRESENT | BMI270_DIAG_CHIPID_OK;
    }
    if (app_vars.bmi270_read_ok==TRUE) {
        app_vars.bmi270_diag |= BMI270_DIAG_READ_OK;
    }
    if (app_vars.bmi270_addr==BMI270_ADDR_ALT) {
        app_vars.bmi270_diag |= BMI270_DIAG_ALT_ADDR;
    }
    if ((app_vars.bmi270_internal_status & 0x0f)==BMI270_INTERNAL_STATUS_INIT_OK) {
        app_vars.bmi270_diag |= BMI270_DIAG_INIT_OK;
    }

    if (app_vars.bmi270_present==TRUE) {
        bmi270_power_down();
    }
    i2c_disable();
}

void update_bmi270_sample(void) {

    bool bmi270_was_present;

    if (g_bmi271_enabled==FALSE) {
        i2c_disable();
        app_vars.bmi270_present = FALSE;
        app_vars.bmi270_read_ok = FALSE;
        app_vars.bmi270_diag = 0;
        app_vars.bmi270_who_am_i = 0;
        app_vars.bmi270_status = 0;
        app_vars.bmi270_error_reg = 0;
        app_vars.bmi270_internal_status = 0;
        app_vars.acc_x = 0;
        app_vars.acc_y = 0;
        app_vars.acc_z = 0;
        return;
    }

    bmi270_was_present = app_vars.bmi270_present;

    if (
        (app_vars.bmi270_present==FALSE) ||
        (app_vars.bmi270_read_ok==FALSE) ||
        ((app_vars.bmi270_internal_status & 0x0f)!=BMI270_INTERNAL_STATUS_INIT_OK)
    ) {
        app_vars.bmi270_addr = BMI270_ADDR;
        i2c_set_addr(app_vars.bmi270_addr);
        app_vars.bmi270_who_am_i = bmi270_who_am_i();
        app_vars.bmi270_read_ok = (bmi270_last_i2c_result()!=0);

        if (app_vars.bmi270_who_am_i!=BMI270_CHIPID) {
            app_vars.bmi270_addr = BMI270_ADDR_ALT;
            i2c_set_addr(app_vars.bmi270_addr);
            app_vars.bmi270_who_am_i = bmi270_who_am_i();
            app_vars.bmi270_read_ok = (bmi270_last_i2c_result()!=0);
        }

        if (app_vars.bmi270_who_am_i==BMI270_CHIPID) {
            app_vars.bmi270_present = (bmi270_default_config()!=0);
            bmi270_was_present = FALSE;
            app_vars.bmi270_status = bmi270_get_status();
            app_vars.bmi270_error_reg = bmi270_get_errorreg();
            app_vars.bmi270_internal_status = bmi270_get_internal_status();
        }
    }

    app_vars.bmi270_diag = 0;
    if (app_vars.bmi270_present==TRUE) {
        app_vars.bmi270_diag |= BMI270_DIAG_PRESENT | BMI270_DIAG_CHIPID_OK;
    }
    if (app_vars.bmi270_addr==BMI270_ADDR_ALT) {
        app_vars.bmi270_diag |= BMI270_DIAG_ALT_ADDR;
    }

    if (app_vars.bmi270_present==FALSE) {
        app_vars.acc_x = 0;
        app_vars.acc_y = 0;
        app_vars.acc_z = 0;
        i2c_disable();
        return;
    }

    i2c_set_addr(app_vars.bmi270_addr);
    if (bmi270_was_present==TRUE) {
        bmi270_power_on();
    }
    app_vars.bmi270_read_ok = (bmi270_read_6dof_data()!=0);

    if (app_vars.bmi270_read_ok==TRUE) {
        app_vars.bmi270_diag |= BMI270_DIAG_READ_OK;
        app_vars.acc_x = bmi270_read_acc_x();
        app_vars.acc_y = bmi270_read_acc_y();
        app_vars.acc_z = bmi270_read_acc_z();
    } else {
        app_vars.acc_x = 0;
        app_vars.acc_y = 0;
        app_vars.acc_z = 0;
    }

    app_vars.bmi270_status = bmi270_get_status();
    app_vars.bmi270_error_reg = bmi270_get_errorreg();
    app_vars.bmi270_internal_status = bmi270_get_internal_status();
    if ((app_vars.bmi270_internal_status & 0x0f)==BMI270_INTERNAL_STATUS_INIT_OK) {
        app_vars.bmi270_diag |= BMI270_DIAG_INIT_OK;
    }
}

void send_next_adv_packet(void) {

    radio_rfOn();

    if (g_bmi271_enabled==TRUE) {
        i2c_init();
    }
    if (g_led_enabled==TRUE) {
        NRF_P1->OUTCLR = (uint32_t)1 << BLE_ADV_LED_PIN;

        NRF_P1->PIN_CNF[BLE_ADV_LED_PIN] =
              ((uint32_t)GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos)
            | ((uint32_t)GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos)
            | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
            | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
            | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);

    }
    update_bmi270_sample();
    assemble_adv_name_packet();

    radio_setFrequency(
        ble_adv_channels[app_vars.adv_channel_index],
        FREQ_TX
    );
    app_vars.adv_channel_index++;

    radio_loadPacket(app_vars.packet,app_vars.packet_len);
    if (g_led_enabled==TRUE) {
        app_vars.led_on = TRUE;
        app_vars.led_off_time = sctimer_readCounter()+BLE_TX_LED_ON_TICKS;
        NRF_P1->OUTSET = (uint32_t)1 << BLE_ADV_LED_PIN;
    }
    radio_txEnable();
    app_vars.state = APP_STATE_TX;
    radio_txNow();
}

//=========================== callbacks =======================================

void cb_startFrame(PORT_TIMER_WIDTH timestamp) {
    // update debug stats
    app_dbg.num_startFrame++;
}

void cb_endFrame(PORT_TIMER_WIDTH timestamp) {
    // set flag
    app_vars.flags |= APP_FLAG_END_FRAME;

    // update debug stats
    app_dbg.num_endFrame++;
}

void cb_timer(void) {
    // set flag
    app_vars.flags |= APP_FLAG_TIMER;

    // update debug stats
    app_dbg.num_timer++;
}

void cb_uartTxDone(void) {

   app_vars.uart_lastTxByteIndex++;
   if (app_vars.uart_lastTxByteIndex<LEN_UART_BUFFER) {
      uart_writeByte(app_vars.uart_buffer_to_send[app_vars.uart_lastTxByteIndex]);
   } else {
      app_vars.uartDone = 1;
   }
}

uint8_t cb_uartRxCb(void) {
   uint8_t byte;
   
   // read received byte
   byte = uart_readByte();
   
   // echo that byte over serial
   uart_writeByte(byte);
   
   return 0;
}
