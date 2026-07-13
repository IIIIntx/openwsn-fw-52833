/**
\brief This program shows the use of the "radio" bsp module.

Since the bsp modules for different platforms have the same declaration, you
can use this project with any platform.

The board running this program will send a packet on channel CHANNEL every
TIMER_PERIOD ticks. The packet contains LENGTH_PACKET bytes. The first byte
is the packet number, which increments for each transmitted packet. The
remainder of the packet contains an incrementing bytes.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, August 2014.
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

//=========================== defines =========================================

#define LENGTH_BLE_CRC  3
#define LENGTH_PACKET   125+LENGTH_BLE_CRC  ///< maximum length is 127 bytes
#define CHANNEL         17              ///< 0~39
#define TIMER_PERIOD    (0xffff>>2)     ///< 0xffff = 2s@32kHz

#define NUM_SAMPLES     SAMPLE_MAXCNT
//#define LEN_UART_BUFFER ((NUM_SAMPLES*4)+8)
#define LEN_UART_BUFFER ((NUM_SAMPLES*4)*2+9)
#define LENGTH_SERIAL_FRAME  127            // length of the serial frame

#define ENABLE_DF       1

#define DEBUG_RADIO_PIN 11
#define PAIR_TIMEOUT_TICKS  ((16000000/10)*4)  // 400 ms @ 16 MHz timer

#define UART_OUTPUT_IQ_FRAME      0   // original 713-byte binary frame
#define UART_OUTPUT_RX_SEQ_TEXT   1   // text lines for serial terminal viewer
#define UART_TEXT_LINE_LEN        16
#define UART_TEXT_QUEUE_LEN       4
#ifndef UART_OUTPUT_MODE
#define UART_OUTPUT_MODE          UART_OUTPUT_RX_SEQ_TEXT
#endif


uint16_t length = 0;

const static uint8_t ble_device_addr[6] = { 
    0xaa, 0xbb, 0xcc, 0xcc, 0xbb, 0xaa
};

// get from https://openuuid.net/signin/:  a24e7112-a03f-4623-bb56-ae67bd653c73
const static uint8_t ble_uuid[16]       = {
    0xa2, 0x4e, 0x71, 0x12, 0xa0, 0x3f, 
    0x46, 0x23, 0xbb, 0x56, 0xae, 0x67,
    0xbd, 0x65, 0x3c, 0x73
};

//=========================== variables =======================================

enum {
    APP_FLAG_START_FRAME = 0x01,
    APP_FLAG_END_FRAME   = 0x02,
    APP_FLAG_TIMER       = 0x04,
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
                uint8_t         flags;
                app_state_t     state;
                
                int8_t          rxpk_rssi;
                uint8_t         rxpk_lqi;
                bool            rxpk_crc;
                uint16_t        num_samples;

                uint8_t         prob;

                uint32_t        tx1_sample_buffer[NUM_SAMPLES];
                uint32_t        tx2_sample_buffer[NUM_SAMPLES];
                
                uint8_t         uart_buffer_to_send[LEN_UART_BUFFER];
                uint8_t         uart_pending_buffer[UART_TEXT_QUEUE_LEN][UART_TEXT_LINE_LEN];

                uint16_t        uart_lastTxByteIndex;
                uint16_t        uart_tx_len;
     volatile   uint8_t         uart_tx_busy;
     volatile   uint8_t         uart_pending_head;
     volatile   uint8_t         uart_pending_tail;
     volatile   uint8_t         uart_pending_count;
     volatile   uint8_t         uartDone;
     volatile   uint8_t         rxpk_done;
                uint8_t         rxpk_buf[LENGTH_PACKET];
                uint8_t         rxpk_freq_offset;
                uint8_t         rxpk_len;
                uint8_t         rxpk_num;

     volatile   bool            tx1_done;
     volatile   uint8_t         tx1_packet_sqn;
     volatile   uint32_t        tx1_done_timestamp;
     volatile   uint16_t        tx1_num_samples;

     volatile   bool            tx2_done;
     volatile   uint8_t         tx2_packet_sqn;
     volatile   uint32_t        tx2_done_timestamp;
     volatile   uint16_t        tx2_num_samples;

                uint32_t        time_interval;
     volatile   uint8_t         pair_pending;
     volatile   uint32_t        pair_start_timestamp;


                uint8_t         uart_txFrame[LENGTH_SERIAL_FRAME];
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);

void     cb_uartTxDone(void);
uint8_t  cb_uartRxCb(void);
void     uart_write_rx_seq_text(uint8_t tx_id, uint8_t seq);

void nrf_gpio_cfg_output(uint8_t port_number, uint32_t pin_number);
//=========================== main ============================================

/**
\brief The program starts executing here.
*/
int mote_main(void) {
    uint16_t i;

    uint8_t freq_offset;
    uint8_t sign;
    uint8_t read;
    
    uint32_t current_ticks;
    uint8_t antenna_id;

    app_vars.prob = 1;

    // clear local variables
    memset(&app_vars,0,sizeof(app_vars_t));

    // initialize board
    board_init();
    leds_init();
    
    timer_init();
    timer_start();
    
#if ENABLE_DF == 1
    //antenna_CHW_rx_switch_init();
    // Single-antenna node: no DFE GPIO antenna switching is required.
    // radio_configure_direction_finding_antenna_switch();
    //set_antenna_CHW_switches();
#endif

    // board_init() deliberately disables UART, so RX1 must initialize it
    // before installing callbacks and starting interrupt-driven transmission.
    uart_init();
    uart_setCallbacks(cb_uartTxDone,cb_uartRxCb);
    uart_enableInterrupts();

    // P0.11 is the UART TX pin.  Do not reuse it as a radio debug GPIO.
    // nrf_gpio_cfg_output(0, DEBUG_RADIO_PIN);

    // add radio callback functions
    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);

    // prepare radio
    radio_rfOn();
    // freq type only effects on scum port
    radio_setFrequency(CHANNEL, FREQ_TX);

#if ENABLE_DF == 1
    radio_configure_direction_finding_manual_AoA();
    memset(app_vars.tx1_sample_buffer,0,sizeof(app_vars.tx1_sample_buffer));
    memset(app_vars.tx2_sample_buffer,0,sizeof(app_vars.tx2_sample_buffer));
    radio_set_df_sample_buffer(app_vars.tx1_sample_buffer,NUM_SAMPLES);
#endif

    // switch in RX by default
    radio_rxEnable();
    radio_rxNow();
    // RX diagnostic LEDs:
    // - error LED toggles when a valid TX1 packet is received.
    // - debug LED toggles when a valid TX2 packet is received.
    // - radio LED toggles when TX1/TX2 sequence numbers are aligned.
    leds_radio_off();
    leds_error_off();
    leds_debug_off();


    while(1) {

        // wait for timer to elapse
        app_vars.rxpk_done = 0;
        while (app_vars.rxpk_done==0) {
            if (app_vars.pair_pending) {
                timer_capture_now(0);
                current_ticks = timer_getCapturedValue(0);
                if ((current_ticks - app_vars.pair_start_timestamp) > PAIR_TIMEOUT_TICKS) {
                    app_vars.tx1_done = 0;
                    app_vars.tx1_packet_sqn = 0;
                    app_vars.tx1_num_samples = 0;
                    app_vars.tx2_done = 0;
                    app_vars.tx2_packet_sqn = 0;
                    app_vars.tx2_num_samples = 0;
                    app_vars.pair_pending = 0;

                    memset(app_vars.tx1_sample_buffer,0,sizeof(app_vars.tx1_sample_buffer));
                    memset(app_vars.tx2_sample_buffer,0,sizeof(app_vars.tx2_sample_buffer));
                    radio_set_df_sample_buffer(app_vars.tx1_sample_buffer,NUM_SAMPLES);
                }
            }
            continue;
        }

        if (app_vars.rxpk_crc && ENABLE_DF) {
            
            if (app_vars.tx1_done && app_vars.tx2_done) {
                if (app_vars.tx1_packet_sqn == app_vars.tx2_packet_sqn) {
                    leds_radio_toggle();

                    if (
                        app_vars.tx1_num_samples == NUM_SAMPLES &&
                        app_vars.tx2_num_samples == NUM_SAMPLES
                    ) {

#if UART_OUTPUT_MODE == UART_OUTPUT_IQ_FRAME
                        for (i=0;i<NUM_SAMPLES;i++) {
                            app_vars.uart_buffer_to_send[4*i+0] = (app_vars.tx1_sample_buffer[i] >>24) & 0x000000ff;
                            app_vars.uart_buffer_to_send[4*i+1] = (app_vars.tx1_sample_buffer[i] >>16) & 0x000000ff;
                            app_vars.uart_buffer_to_send[4*i+2] = (app_vars.tx1_sample_buffer[i] >> 8) & 0x000000ff;
                            app_vars.uart_buffer_to_send[4*i+3] = (app_vars.tx1_sample_buffer[i] >> 0) & 0x000000ff;
                        }

                        for (i=0;i<NUM_SAMPLES;i++) {
                            app_vars.uart_buffer_to_send[4*i+0 + 352] = (app_vars.tx2_sample_buffer[i] >>24) & 0x000000ff;
                            app_vars.uart_buffer_to_send[4*i+1 + 352] = (app_vars.tx2_sample_buffer[i] >>16) & 0x000000ff;
                            app_vars.uart_buffer_to_send[4*i+2 + 352] = (app_vars.tx2_sample_buffer[i] >> 8) & 0x000000ff;
                            app_vars.uart_buffer_to_send[4*i+3 + 352] = (app_vars.tx2_sample_buffer[i] >> 0) & 0x000000ff;
                        }
                        
                        app_vars.time_interval = app_vars.tx2_done_timestamp - app_vars.tx1_done_timestamp;

                        app_vars.uart_buffer_to_send[704] = (app_vars.time_interval >> 24) & 0x000000ff;
                        app_vars.uart_buffer_to_send[705] = (app_vars.time_interval >> 16) & 0x000000ff;
                        app_vars.uart_buffer_to_send[706] = (app_vars.time_interval >>  8) & 0x000000ff;
                        app_vars.uart_buffer_to_send[707] = (app_vars.time_interval >>  0) & 0x000000ff;

                        app_vars.uart_buffer_to_send[708] = app_vars.tx1_packet_sqn;
                        app_vars.uart_buffer_to_send[709] = app_vars.tx2_packet_sqn;

                        app_vars.uart_buffer_to_send[710]     = 0xff;
                        app_vars.uart_buffer_to_send[711]     = 0xff; 
                        app_vars.uart_buffer_to_send[712]     = 0xff;

                        app_vars.uart_tx_len = LEN_UART_BUFFER;
                        app_vars.uart_lastTxByteIndex = 0;
                        app_vars.uart_tx_busy = 1;
                        uart_writeByte(app_vars.uart_buffer_to_send[0]);
#endif // UART_OUTPUT_MODE == UART_OUTPUT_IQ_FRAME

                        app_vars.tx1_done = 0;
                        app_vars.tx1_packet_sqn = 0;
                        app_vars.tx1_num_samples = 0;
                        app_vars.tx2_done = 0;
                        app_vars.tx2_packet_sqn = 0;
                        app_vars.tx2_num_samples = 0;
                    }
                } 
                app_vars.tx1_done = 0;
                app_vars.tx1_packet_sqn = 0;
                app_vars.tx1_num_samples = 0;
                app_vars.tx2_done = 0;
                app_vars.tx2_packet_sqn = 0;
                app_vars.tx2_num_samples = 0;
                app_vars.pair_pending = 0;

                memset(app_vars.tx1_sample_buffer,0,sizeof(app_vars.tx1_sample_buffer));
                memset(app_vars.tx2_sample_buffer,0,sizeof(app_vars.tx2_sample_buffer));
                radio_set_df_sample_buffer(app_vars.tx1_sample_buffer,NUM_SAMPLES);

            }

           

        }

    }
}

//=========================== private =========================================

void uart_write_rx_seq_text(uint8_t tx_id, uint8_t seq) {

    uint8_t* buffer;

    if (app_vars.uart_tx_busy) {
        if (app_vars.uart_pending_count >= UART_TEXT_QUEUE_LEN) {
            return;
        }
        buffer = app_vars.uart_pending_buffer[app_vars.uart_pending_tail];
        app_vars.uart_pending_tail++;
        if (app_vars.uart_pending_tail >= UART_TEXT_QUEUE_LEN) {
            app_vars.uart_pending_tail = 0;
        }
        app_vars.uart_pending_count++;
    } else {
        buffer = app_vars.uart_buffer_to_send;
        app_vars.uart_tx_len = UART_TEXT_LINE_LEN;
        app_vars.uart_lastTxByteIndex = 0;
        app_vars.uart_tx_busy = 1;
    }

    buffer[0]  = 'R';
    buffer[1]  = 'X';
    buffer[2]  = ' ';
    buffer[3]  = 'T';
    buffer[4]  = 'X';
    buffer[5]  = '0' + tx_id;
    buffer[6]  = ' ';
    buffer[7]  = 's';
    buffer[8]  = 'e';
    buffer[9]  = 'q';
    buffer[10] = '=';
    buffer[11] = '0' + (seq / 100);
    buffer[12] = '0' + ((seq / 10) % 10);
    buffer[13] = '0' + (seq % 10);
    buffer[14] = '\r';
    buffer[15] = '\n';

    if (buffer == app_vars.uart_buffer_to_send) {
        uart_writeByte(app_vars.uart_buffer_to_send[0]);
    }
}

//=========================== callbacks =======================================

void cb_startFrame(PORT_TIMER_WIDTH timestamp) {
    // set flag
    //app_vars.flags |= APP_FLAG_START_FRAME;

    //leds_sync_on();
    // update debug stats
    app_dbg.num_startFrame++;
}

void cb_endFrame(PORT_TIMER_WIDTH timestamp) {

    //NRF_P0->OUTSET =  1 << DEBUG_RADIO_PIN;
    //NRF_P0->OUTCLR =  1 << DEBUG_RADIO_PIN;

    bool     expectedFrame;
    //uint8_t  i;

    // update debug stats
    app_dbg.num_endFrame++;

    // Keep the indicator on: RX1 remains a continuous logical listener while
    // this completed frame is processed and the radio is re-armed below.
    
    memset(&app_vars.rxpk_buf[0],0,LENGTH_PACKET);
    
    app_vars.rxpk_len = sizeof(app_vars.rxpk_buf);

    radio_getReceivedFrame(
        app_vars.rxpk_buf,
        &app_vars.rxpk_len,
        sizeof(app_vars.rxpk_buf),
        &app_vars.rxpk_rssi,
        &app_vars.rxpk_lqi,
        &app_vars.rxpk_crc
    );

    // check the frame is sent by radio_tx project
    expectedFrame = TRUE;
    
    if (
        !app_vars.rxpk_crc ||
        app_vars.rxpk_len <= 34 ||
        app_vars.rxpk_len > LENGTH_PACKET
    ) {
        expectedFrame = FALSE;
    } else {

        if(app_vars.rxpk_buf[0]!=0x42){
            expectedFrame = FALSE;
        }
    }
    
    if (expectedFrame){

        if (
            (app_vars.rxpk_buf[34] == 1 || app_vars.rxpk_buf[34] == 2) &&
            !app_vars.pair_pending
        ) {
            app_vars.pair_pending = 1;
            app_vars.pair_start_timestamp = timestamp;
        }

        if (app_vars.rxpk_buf[34] == 1) {
            // TX1 is the start of a new pairing round.  Any stored TX2 before
            // this TX1 belongs to an older round and must not be matched with
            // the current TX1.
            app_vars.tx2_done = 0;
            app_vars.tx2_packet_sqn = 0;
            app_vars.tx2_num_samples = 0;
            app_vars.pair_pending = 1;
            app_vars.pair_start_timestamp = timestamp;

            app_vars.tx1_done = 1;
            app_vars.tx1_done_timestamp = timer_getCapturedValue(1);
            app_vars.tx1_packet_sqn = app_vars.rxpk_buf[33];
            app_vars.tx1_num_samples = radio_get_df_sample_amount();
            // Do not copy IQ here.  TX1 IQ was written directly into
            // tx1_sample_buffer by RADIO DFEPACKET.PTR.  Point the next RX
            // capture at tx2_sample_buffer and re-enter RX immediately.
            radio_set_df_sample_buffer(app_vars.tx2_sample_buffer,NUM_SAMPLES);
            radio_rxEnable();
            radio_rxNow();

            leds_error_toggle();
#if UART_OUTPUT_MODE == UART_OUTPUT_RX_SEQ_TEXT
            uart_write_rx_seq_text(1, app_vars.tx1_packet_sqn);
#endif
        }

        if (app_vars.rxpk_buf[34] == 2) {
            leds_debug_toggle();
#if UART_OUTPUT_MODE == UART_OUTPUT_RX_SEQ_TEXT
            uart_write_rx_seq_text(2, app_vars.rxpk_buf[33]);
#endif
            app_vars.tx2_done = 1;
            app_vars.tx2_done_timestamp = timer_getCapturedValue(1);
            app_vars.tx2_packet_sqn = app_vars.rxpk_buf[33];
            app_vars.tx2_num_samples = radio_get_df_sample_amount();
            app_vars.num_samples = NUM_SAMPLES;
            // The next useful frame should be a new TX1 packet.
            radio_set_df_sample_buffer(app_vars.tx1_sample_buffer,NUM_SAMPLES);

            app_vars.rxpk_done = 1;
        }

        //app_vars.rxpk_done = 1;
    }
    //leds_debug_toggle();

    // keep listening (needed for at86rf215 radio)
    radio_rxEnable();
    radio_rxNow();
    // Keep radio_rxEnable() from turning LED2 permanently on; RX diagnostics
    // use LED1/LED3 above.
    leds_radio_off();

    // led
    //leds_sync_off();

}

void cb_uartTxDone(void) {

   app_vars.uart_lastTxByteIndex++;
   if (app_vars.uart_lastTxByteIndex<app_vars.uart_tx_len) {
      uart_writeByte(app_vars.uart_buffer_to_send[app_vars.uart_lastTxByteIndex]);
   } else {
      app_vars.uartDone = 1;
      if (app_vars.uart_pending_count) {
         memcpy(app_vars.uart_buffer_to_send,app_vars.uart_pending_buffer[app_vars.uart_pending_head],UART_TEXT_LINE_LEN);
         app_vars.uart_tx_len = UART_TEXT_LINE_LEN;
         app_vars.uart_lastTxByteIndex = 0;
         app_vars.uart_pending_head++;
         if (app_vars.uart_pending_head >= UART_TEXT_QUEUE_LEN) {
            app_vars.uart_pending_head = 0;
         }
         app_vars.uart_pending_count--;
         app_vars.uart_tx_busy = 1;
         uart_writeByte(app_vars.uart_buffer_to_send[0]);
      } else {
         app_vars.uart_tx_busy = 0;
      }
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
