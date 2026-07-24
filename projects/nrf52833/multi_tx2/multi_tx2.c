/**
\brief This program is the tx2 for AMUNA
\author Manjiang Cao <mcao999@connect.hkust-gz.edu.cn>, Sept 2025.
*/


#include "stdint.h"
#include "string.h"
#include "board.h"
#include "radio.h"
#include "leds.h"
#include "sctimer.h"
#include "radio_df.h"
#include "aod.h"
#include "uart.h"
#include "timer.h"
#include "debugpins.h"

//=========================== defines =========================================

#define LENGTH_BLE_CRC  3
#define LENGTH_PACKET   125+LENGTH_BLE_CRC  ///< maximum length is 127 bytes
#define CHANNEL         17              ///< 0~39
#define TIMER_PERIOD    (0xffff>>2)     ///< 0xffff = 2s@32kHz

#define NUM_SAMPLES     SAMPLE_MAXCNT
#define LEN_UART_BUFFER ((NUM_SAMPLES*4)+8)
//#define LEN_UART_BUFFER ((NUM_SAMPLES*4)*2+7)
#define LENGTH_SERIAL_FRAME  127            // length of the serial frame

#define ENABLE_DF       1


const static uint8_t ble_device_addr[6] = { 
    0xaa, 0xbb, 0xcc, 0xcc, 0xbb, 0xaa
};

// get from https://openuuid.net/signin/:  a24e7112-a03f-4623-bb56-ae67bd653c73
const static uint8_t ble_uuid[16]       = {
    0xa2, 0x4e, 0x71, 0x12, 0xa0, 0x3f, 
    0x46, 0x23, 0xbb, 0x56, 0xae, 0x67,
    0xbd, 0x65, 0x3c, 0x73
};

#define DEBUG_RADIO_PIN 11

#define SEND_PERIOD_TICKS       (16000000/1000)*500  // TX2 start-to-start: 500 ms
#define TX1_TO_TX2_START_TICKS  (16000000/1000)*5    // on-air start-to-start: 5 ms
#define BLE1M_ADDRESS_END_TICKS (16000000/1000000)*40 // 8 us preamble + 32 us access address
#define SYNC_OFFSET_TICKS       (TX1_TO_TX2_START_TICKS-BLE1M_ADDRESS_END_TICKS)
#define TX_PREPARE_ADVANCE      (16000000/1000)*1    // prepare autonomous TX 1 ms early
#define TX2_AUTONOMOUS_AFTER_SYNC 1

//=========================== variables =======================================

typedef enum {
    APP_STATE_TX          = 0x01,
    APP_STATE_RX          = 0x02,
    APP_STATE_OFF         = 0x04,
} app_state_t;

typedef struct {
    uint8_t              num_startFrame;
    uint8_t              num_endFrame;
    uint8_t              num_timer;
} app_dbg_t;

app_dbg_t app_dbg;

typedef struct {

                uint8_t         slot_timerId;
                uint8_t         inner_timerId;
                app_state_t     state;

                uint8_t         slot_offset;
                uint8_t         pkt_sqn;
                uint32_t        time_slotStartAt;

                uint8_t         packet[LENGTH_PACKET];
                uint8_t         packet_len;
                uint8_t         rxpk_packet[LENGTH_PACKET];
                uint8_t         rxpk_packet_len;
                int8_t          rxpk_rssi;
                uint8_t         rxpk_lqi;
                bool            rxpk_crc;

                uint8_t         rx_doneAt;
                uint8_t         tx_now;

                uint32_t       start_timestamp;
                uint32_t       end_timestamp;
                uint32_t       time_interval;

                bool           isTargetPkt;
                bool           synced;

} app_vars_t;

app_vars_t app_vars;
//=========================== prototypes ======================================

void      cb_startFrame(PORT_TIMER_WIDTH timestamp);
void      cb_endFrame(PORT_TIMER_WIDTH timestamp);

void      cb_timer(void);
void      assemble_ibeacon_packet(uint8_t);
void      prepare_tx_packet(uint8_t sqn);
void      schedule_next_synced_tx(void);
void      nrf_gpio_cfg_output(uint8_t port_number, uint32_t pin_number);
//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {
    // clear local variables
    memset(&app_vars,0,sizeof(app_vars_t));

    // initialize board
    board_init();
    leds_init();
    debugpins_init();

    radio_rfOff();
    app_vars.state = APP_STATE_OFF;

    nrf_gpio_cfg_output(0, DEBUG_RADIO_PIN);

#if ENABLE_DF == 1
    //antenna_CHW_rx_switch_init();
    // Single-antenna node: no DFE GPIO antenna switching is required.
    // radio_configure_direction_finding_antenna_switch();
    // Use AoA consistently with the single-antenna CTE transmitters.
    radio_configure_direction_finding_manual_AoA();
    //set_antenna_CHW_switches();
#endif

    // add radio callback functions
    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);
    
    //sctimer_set_callback(0, cb_timer);
    //timer_capture_now(0);
    //app_vars.time_slotStartAt = sctimer_readCounter() + SEND_DURATION;
    //sctimer_setCompare(0, app_vars.time_slotStartAt);

    timer_init();
    timer_start();

    timer_set_callback(0, cb_timer);
    timer_capture_now(0);
    app_vars.time_slotStartAt = timer_getCapturedValue(0) + SEND_PERIOD_TICKS;
    timer_schedule(0, app_vars.time_slotStartAt - TX_PREPARE_ADVANCE);
    // Start autonomously.  A later valid TX1 reception replaces this deadline
    // and synchronizes both the clock phase and packet sequence number.
    app_vars.synced = TRUE;

    // prepare radio
    radio_rfOn();
    // freq type only effects on scum port
    radio_setFrequency(CHANNEL, FREQ_RX);

    radio_rxEnable();
    app_vars.state = APP_STATE_RX;
    radio_rxNow();
    // Error LED indicates that RX listening is active.  Keep the radio LED
    // off so its TX toggle is visible when a response packet is sent.
    leds_radio_off();
    leds_error_on();

    while(1) {
        board_sleep();
    }
}

//=========================== private =========================================

void assemble_ibeacon_packet(uint8_t sqn) {

     uint8_t i;
     i=0;

     memset( app_vars.packet, 0x00, sizeof(app_vars.packet) );

     app_vars.packet[i++]  = 0x42;               // BLE ADV_NONCONN_IND (this is a must)
     app_vars.packet[i++]  = 0x21;               // Payload length
     app_vars.packet[i++]  = ble_device_addr[0]; // BLE adv address byte 0
     app_vars.packet[i++]  = ble_device_addr[1]; // BLE adv address byte 1
     app_vars.packet[i++]  = ble_device_addr[2]; // BLE adv address byte 2
     app_vars.packet[i++]  = ble_device_addr[3]; // BLE adv address byte 3
     app_vars.packet[i++]  = ble_device_addr[4]; // BLE adv address byte 4
     app_vars.packet[i++]  = ble_device_addr[5]; // BLE adv address byte 5

     app_vars.packet[i++]  = 0x1a;
     app_vars.packet[i++]  = 0xff;
     app_vars.packet[i++]  = 0x4c;
     app_vars.packet[i++]  = 0x00;

     app_vars.packet[i++]  = 0x02;
     app_vars.packet[i++]  = 0x15;
     memcpy(&app_vars.packet[i], &ble_uuid[0], 16);
     i                    += 16;
     app_vars.packet[i++]  = 0x00;               // major
     app_vars.packet[i++]  = 0xff;
     app_vars.packet[i++]  = 0x00;               // minor
     app_vars.packet[i++]  = sqn;                // 34 byte
     app_vars.packet[i++]  = 0x02;               // tx id
}

void prepare_tx_packet(uint8_t sqn) {
    app_vars.packet_len = sizeof(app_vars.packet);

    radio_rfOn();
    radio_setFrequency(CHANNEL, FREQ_TX);
    assemble_ibeacon_packet(sqn);
    radio_loadPacket(app_vars.packet, LENGTH_PACKET);

    // Single-antenna node: keep DFE GPIO antenna switching disabled.
    // radio_configure_direction_finding_antenna_switch();
    // AoA TX generates the CTE without transmitter-side antenna switching.
    radio_configure_direction_finding_manual_AoA();

    radio_txEnable();
    app_vars.state = APP_STATE_TX;
    leds_radio_off();
    leds_error_off();
}

void schedule_next_synced_tx(void) {
    app_vars.pkt_sqn++;
#if TX2_AUTONOMOUS_AFTER_SYNC
    // Keep TX2 start-to-start spacing at exactly 500 ms when TX1 is missed.
    app_vars.time_slotStartAt += SEND_PERIOD_TICKS;
    timer_schedule(0, app_vars.time_slotStartAt - TX_PREPARE_ADVANCE);
#endif
}
//=========================== callbacks =======================================

void cb_startFrame(PORT_TIMER_WIDTH timestamp) {
    // set flag
    //app_vars.flags |= APP_FLAG_START_FRAME;

    //leds_sync_on();
    // update debug stats
    app_dbg.num_startFrame++;
    app_vars.start_timestamp = timestamp;
}

void cb_endFrame(PORT_TIMER_WIDTH timestamp) {
    app_dbg.num_endFrame++;
    
    radio_rfOff();
    // A completed RX frame or TX frame means this node is no longer in the
    // active listening portion of the state machine.
    leds_error_off();

    if (app_vars.state == APP_STATE_RX) {
        
        app_vars.isTargetPkt = FALSE;
        
        radio_getReceivedFrame(
            app_vars.rxpk_packet,
            &app_vars.rxpk_packet_len,
            sizeof(app_vars.rxpk_packet),
            &app_vars.rxpk_rssi,
            &app_vars.rxpk_lqi,
            &app_vars.rxpk_crc
        );
        
        if (
            app_vars.rxpk_crc &&
            app_vars.rxpk_packet_len >= 35 &&
            app_vars.rxpk_packet[0] == 0x42 &&
            app_vars.rxpk_packet[1] == 0x21 &&
            app_vars.rxpk_packet[34] == 0x01
        ) {
            app_vars.isTargetPkt = TRUE;      //Check if received packet is a legal plast system packet
        }

        if (app_vars.isTargetPkt) {
            // ADDRESS is raised at the end of the BLE preamble/access address,
            // 40 us after TX1 begins on air.  Subtract that delay so TX2 starts
            // 5 ms after the actual TX1 on-air start, independent of packet length.
            app_vars.synced = TRUE;
            app_vars.time_slotStartAt = app_vars.start_timestamp + SYNC_OFFSET_TICKS;
            timer_schedule(0, app_vars.time_slotStartAt);
            
            app_vars.pkt_sqn = app_vars.rxpk_packet[33];
            prepare_tx_packet(app_vars.pkt_sqn);
            return;
        } else {
            radio_rfOn();
            radio_configure_direction_finding_manual_AoA();
            radio_setFrequency(CHANNEL, FREQ_RX);
            radio_rxEnable();
            radio_rxNow();
            // Listening has resumed after rejecting this packet.
            leds_radio_off();
            leds_error_on();
        }
    }

    if (app_vars.state == APP_STATE_TX) {
#if TX2_AUTONOMOUS_AFTER_SYNC
        if (app_vars.synced) {
            schedule_next_synced_tx();
        }
#endif

        radio_rfOn();
        radio_setFrequency(CHANNEL, FREQ_RX);
        radio_configure_direction_finding_manual_AoA();
        radio_rxEnable();
        app_vars.state = APP_STATE_RX;
        radio_rxNow();
        // Response TX has ended and listening is active again.
        leds_radio_off();
        leds_error_on();
    }
}

void cb_timer(void) {
    app_dbg.num_timer++;

    if (app_vars.state == APP_STATE_TX) {
        // TX is already prepared.  This compare event is the exact TX start.
        leds_radio_toggle();
        radio_txNow();
#if TX2_AUTONOMOUS_AFTER_SYNC
    } else if (app_vars.state == APP_STATE_RX && app_vars.synced) {
        // Autonomous TX path.  TX2 did not receive a fresh TX1 in this round,
        // so prepare its predicted seq shortly before the scheduled TX time.
        radio_rfOff();
        prepare_tx_packet(app_vars.pkt_sqn);
        timer_schedule(0, app_vars.time_slotStartAt);
#endif
    }
}
