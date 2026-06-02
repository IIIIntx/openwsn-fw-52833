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
#include "radio.h"
#include "leds.h"
#include "sctimer.h"
#include "uart.h"
#include "i2c.h"
#include "radio_df.h"
#include "bmi270.h"

//=========================== defines =========================================

#define LENGTH_PACKET   38             ///< S0 + length + BLE advertising payload
#define ADV_PERIOD          32768       ///< 1s @32kHz
#define RADIO_BLINK_PERIOD  (0xffff>>6)

#define NUM_SAMPLES     SAMPLE_MAXCNT
#define LEN_UART_BUFFER ((NUM_SAMPLES*4)+8)

#define ENABLE_DF       0

#define BMI270_DIAG_PRESENT       0x01
#define BMI270_DIAG_READ_OK       0x02
#define BMI270_DIAG_CHIPID_OK     0x04
#define BMI270_DIAG_ALT_ADDR      0x08
#define BMI270_DIAG_INIT_OK       0x10

const static uint8_t ble_device_addr[6] = { 
    0xaa, 0xbb, 0xcc, 0xcc, 0xbb, 0xea
};

const static uint8_t ble_adv_channels[3] = {
    37, 38, 39
};

const static uint8_t ble_device_name[] = {
    'n', 'o', 'r', 'd', 'i', 'c'
};

//=========================== variables =======================================

enum {
    APP_FLAG_START_FRAME = 0x01,
    APP_FLAG_END_FRAME   = 0x02,
    APP_FLAG_TIMER       = 0x04,
    APP_FLAG_SEND_NEXT   = 0x08,
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
                bool            radio_led_blinking;
                bool            bmi270_present;
                bool            bmi270_read_ok;
                uint8_t         bmi270_addr;
                uint8_t         bmi270_diag;
                uint8_t         bmi270_who_am_i;
                uint8_t         bmi270_status;
                uint8_t         bmi270_error_reg;
                uint8_t         bmi270_internal_status;
                int16_t         acc_x;
                int16_t         acc_y;
                int16_t         acc_z;
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
void     init_bmi270(void);
void     update_bmi270_sample(void);
void     send_next_adv_packet(void);

//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {
    uint16_t i;

    // clear local variables
    memset(&app_vars,0,sizeof(app_vars_t));

    // initialize board
    board_init();

#if ENABLE_DF == 1
    radio_configure_direction_finding_antenna_switch();

#endif
    uart_setCallbacks(cb_uartTxDone,cb_uartRxCb);
    uart_enableInterrupts();

    init_bmi270();

    // add callback functions radio
    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);

    // start bsp timer
    sctimer_set_callback(cb_timer);
    sctimer_setCompare(sctimer_readCounter()+ADV_PERIOD);
    sctimer_enable();

    // prepare radio
    radio_rfOn();

#if ENABLE_DF == 1
    radio_configure_direction_finding_manual();
#endif

    // stay idle between advertising packets
    app_vars.state = APP_STATE_RX;

    // start by a transmit
    app_vars.flags |= APP_FLAG_TIMER;

    while (1) {

        // sleep while waiting for at least one of the flags to be set
        while (app_vars.flags==0x00) {
            board_sleep();
        }

        // handle and clear every flag
        while (app_vars.flags) {

            //==== APP_FLAG_START_FRAME (TX or RX)
            if (app_vars.flags & APP_FLAG_START_FRAME) {
                // start of frame

                switch (app_vars.state) {
                    case APP_STATE_RX:
                        // started receiving a packet

                        // led
                        leds_error_on();
                        break;
                    case APP_STATE_TX:
                        // started sending a packet

                        // led
                        leds_sync_on();
                    break;
                }

                // clear flag
                app_vars.flags &= ~APP_FLAG_START_FRAME;
            }

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

                        // led
                        leds_error_off();
          
                        // continue to listen
                        radio_rxEnable();
                        radio_rxNow();
                        break;
                    case APP_STATE_TX:
                        // done sending a packet

                        if (app_vars.adv_channel_index<sizeof(ble_adv_channels)) {
                            app_vars.flags |= APP_FLAG_SEND_NEXT;
                        } else {
                            // show one visible P1.09 pulse after the 37/38/39 advertising event
                            app_vars.state = APP_STATE_RX;
                            app_vars.adv_channel_index = 0;
                            leds_radio_on();
                            app_vars.radio_led_blinking = TRUE;
                            sctimer_setCompare(sctimer_readCounter()+RADIO_BLINK_PERIOD);
                            // led
                            leds_sync_off();
                        }
                        break;
                }
                // clear flag
                app_vars.flags &= ~APP_FLAG_END_FRAME;
            }

            //==== APP_FLAG_SEND_NEXT

            if (app_vars.flags & APP_FLAG_SEND_NEXT) {
                send_next_adv_packet();

                // clear flag
                app_vars.flags &= ~APP_FLAG_SEND_NEXT;
            }

            //==== APP_FLAG_TIMER

            if (app_vars.flags & APP_FLAG_TIMER) {
                // timer fired

                if (app_vars.radio_led_blinking==TRUE) {
                    leds_radio_off();
                    app_vars.radio_led_blinking = FALSE;
                    sctimer_setCompare(sctimer_readCounter()+ADV_PERIOD);
                } else if (app_vars.state==APP_STATE_RX) {
                    app_vars.adv_channel_index = 0;
                    update_bmi270_sample();
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

    app_vars.packet_len   = i;
}

void init_bmi270(void) {

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
        app_vars.bmi270_present = TRUE;
        bmi270_default_config();
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
}

void update_bmi270_sample(void) {

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
        return;
    }

    i2c_set_addr(app_vars.bmi270_addr);
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

    assemble_adv_name_packet();

    radio_setFrequency(
        ble_adv_channels[app_vars.adv_channel_index],
        FREQ_TX
    );
    app_vars.adv_channel_index++;

    radio_loadPacket(app_vars.packet,app_vars.packet_len);
    radio_txEnable();
    leds_radio_off();
    app_vars.state = APP_STATE_TX;
    radio_txNow();
}

//=========================== callbacks =======================================

void cb_startFrame(PORT_TIMER_WIDTH timestamp) {
    // set flag
    app_vars.flags |= APP_FLAG_START_FRAME;

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
