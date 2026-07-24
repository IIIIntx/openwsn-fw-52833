/**
\brief RX node for synchronized TX1/TX2 IQ capture.

RX1 continuously listens on CHANNEL.  It stores IQ samples for TX1 and TX2
into separate buffers, reports received packet seq/sample counts over UART in
TEXT mode, and emits the original 713-byte binary IQ frame in IQ_FRAME mode.
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
#define LENGTH_PACKET   125+LENGTH_BLE_CRC
#define CHANNEL         17

#define NUM_SAMPLES     SAMPLE_MAXCNT
#define LEN_UART_BUFFER ((NUM_SAMPLES*4)*2+9)
#define LENGTH_SERIAL_FRAME  127

#define ENABLE_DF       1
#define PAIR_TIMEOUT_TICKS  ((16000000/1000)*100) // process after 100 ms without a target packet

#define UART_OUTPUT_IQ_FRAME      0   // original 713-byte binary frame
#define UART_OUTPUT_RX_SEQ_TEXT   1   // text lines for serial terminal viewer
#define UART_TEXT_LINE_LEN        23   // "RX TX1 seq=123 iq=088\r\n"
#define UART_TEXT_QUEUE_LEN       4
#ifndef UART_OUTPUT_MODE
#define UART_OUTPUT_MODE          UART_OUTPUT_RX_SEQ_TEXT
#endif

//=========================== variables =======================================

typedef struct {
    uint8_t              num_startFrame;
    uint8_t              num_endFrame;
} app_dbg_t;

app_dbg_t app_dbg;

typedef struct {
                int8_t          rxpk_rssi;
                uint8_t         rxpk_lqi;
                bool            rxpk_crc;
                uint8_t         rxpk_buf[LENGTH_PACKET];
                uint8_t         rxpk_len;

                uint32_t        tx1_sample_buffer[NUM_SAMPLES];
                uint32_t        tx2_sample_buffer[NUM_SAMPLES];
                uint32_t        rx_sample_buffer[NUM_SAMPLES];

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
     volatile   uint8_t         led2_match_state;

                uint8_t         uart_txFrame[LENGTH_SERIAL_FRAME];
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void     cb_startFrame(PORT_TIMER_WIDTH timestamp);
void     cb_endFrame(PORT_TIMER_WIDTH timestamp);
void     cb_uartTxDone(void);
uint8_t  cb_uartRxCb(void);

void     uart_write_rx_seq_text(uint8_t tx_id, uint8_t seq, uint16_t iq_num);
void     clear_pair_state(void);

//=========================== main ============================================

int mote_main(void) {
#if UART_OUTPUT_MODE == UART_OUTPUT_IQ_FRAME
    uint16_t i;
#endif
    uint32_t current_ticks;

    memset(&app_vars,0,sizeof(app_vars_t));

    board_init();
    leds_init();

    timer_init();
    timer_start();

#if ENABLE_DF == 1
    // Single-antenna node: no DFE GPIO antenna switching is required.
    // antenna_CHW_rx_switch_init();
    // radio_configure_direction_finding_antenna_switch();
    // set_antenna_CHW_switches();
#endif

    // board_init() deliberately disables UART, so RX1 must initialize it.
    uart_init();
    uart_setCallbacks(cb_uartTxDone,cb_uartRxCb);
    uart_enableInterrupts();

    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);

    radio_rfOn();
    radio_setFrequency(CHANNEL, FREQ_TX);

#if ENABLE_DF == 1
    radio_configure_direction_finding_manual_AoA();
    memset(app_vars.tx1_sample_buffer,0,sizeof(app_vars.tx1_sample_buffer));
    memset(app_vars.tx2_sample_buffer,0,sizeof(app_vars.tx2_sample_buffer));
    memset(app_vars.rx_sample_buffer,0,sizeof(app_vars.rx_sample_buffer));
    radio_set_df_sample_buffer(app_vars.rx_sample_buffer,NUM_SAMPLES);
#endif

    radio_rxEnable();
    radio_rxNow();

    // LED meanings:
    // LED1/error: valid TX1 packet received.
    // LED3/debug: valid TX2 packet received.
    // LED2/radio: seq matched and both IQ sample counts are complete.
    leds_radio_off();
    leds_error_off();
    leds_debug_off();

    while(1) {
        app_vars.rxpk_done = 0;

        while (app_vars.rxpk_done == 0) {
            if (app_vars.pair_pending) {
                timer_capture_now(0);
                current_ticks = timer_getCapturedValue(0);
                if ((current_ticks - app_vars.pair_start_timestamp) > PAIR_TIMEOUT_TICKS) {
                    // No TX1/TX2 packet for 100 ms: close and process this round.
                    app_vars.pair_pending = 0;
                    app_vars.rxpk_done = 1;
                }
            }
        }

        if (ENABLE_DF && (app_vars.tx1_done || app_vars.tx2_done)) {
#if UART_OUTPUT_MODE == UART_OUTPUT_RX_SEQ_TEXT
            if (app_vars.tx1_done) {
                uart_write_rx_seq_text(1, app_vars.tx1_packet_sqn, app_vars.tx1_num_samples);
            }
            if (app_vars.tx2_done) {
                uart_write_rx_seq_text(2, app_vars.tx2_packet_sqn, app_vars.tx2_num_samples);
            }
#endif

            if (app_vars.tx1_done && app_vars.tx2_done) {
                if (
                    app_vars.tx1_packet_sqn == app_vars.tx2_packet_sqn &&
                    app_vars.tx1_num_samples == NUM_SAMPLES &&
                    app_vars.tx2_num_samples == NUM_SAMPLES
                ) {
                    app_vars.led2_match_state = !app_vars.led2_match_state;
                    if (app_vars.led2_match_state) {
                        leds_radio_on();
                    } else {
                        leds_radio_off();
                    }

#if UART_OUTPUT_MODE == UART_OUTPUT_IQ_FRAME
                    if (!app_vars.uart_tx_busy) {
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
                        app_vars.uart_buffer_to_send[710] = 0xff;
                        app_vars.uart_buffer_to_send[711] = 0xff;
                        app_vars.uart_buffer_to_send[712] = 0xff;

                        app_vars.uart_tx_len = LEN_UART_BUFFER;
                        app_vars.uart_lastTxByteIndex = 0;
                        app_vars.uart_tx_busy = 1;
                        uart_writeByte(app_vars.uart_buffer_to_send[0]);
                    }
#endif // UART_OUTPUT_MODE == UART_OUTPUT_IQ_FRAME
                }

            }

            clear_pair_state();
        }
    }
}

//=========================== private =========================================

void clear_pair_state(void) {
    app_vars.tx1_done = 0;
    app_vars.tx1_packet_sqn = 0;
    app_vars.tx1_num_samples = 0;
    app_vars.tx2_done = 0;
    app_vars.tx2_packet_sqn = 0;
    app_vars.tx2_num_samples = 0;
    app_vars.pair_pending = 0;

    memset(app_vars.tx1_sample_buffer,0,sizeof(app_vars.tx1_sample_buffer));
    memset(app_vars.tx2_sample_buffer,0,sizeof(app_vars.tx2_sample_buffer));
    memset(app_vars.rx_sample_buffer,0,sizeof(app_vars.rx_sample_buffer));
    radio_set_df_sample_buffer(app_vars.rx_sample_buffer,NUM_SAMPLES);
}

void uart_write_rx_seq_text(uint8_t tx_id, uint8_t seq, uint16_t iq_num) {
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
    buffer[14] = ' ';
    buffer[15] = 'i';
    buffer[16] = 'q';
    buffer[17] = '=';
    buffer[18] = '0' + ((iq_num / 100) % 10);
    buffer[19] = '0' + ((iq_num / 10) % 10);
    buffer[20] = '0' + (iq_num % 10);
    buffer[21] = '\r';
    buffer[22] = '\n';

    if (buffer == app_vars.uart_buffer_to_send) {
        uart_writeByte(app_vars.uart_buffer_to_send[0]);
    }
}

//=========================== callbacks =======================================

void cb_startFrame(PORT_TIMER_WIDTH timestamp) {
    app_dbg.num_startFrame++;
}

void cb_endFrame(PORT_TIMER_WIDTH timestamp) {
    bool expectedFrame;
    uint16_t received_samples;

    app_dbg.num_endFrame++;

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

    expectedFrame = TRUE;
    if (
        !app_vars.rxpk_crc ||
        app_vars.rxpk_len <= 34 ||
        app_vars.rxpk_len > LENGTH_PACKET ||
        app_vars.rxpk_buf[0] != 0x42 ||
        app_vars.rxpk_buf[1] != 0x21
    ) {
        expectedFrame = FALSE;
    }

    if (expectedFrame) {
        if (
            (app_vars.rxpk_buf[34] == 1 || app_vars.rxpk_buf[34] == 2) &&
            !app_vars.pair_pending
        ) {
            app_vars.pair_pending = 1;
            app_vars.pair_start_timestamp = timestamp;
        }

        if (app_vars.rxpk_buf[34] == 1) {
            received_samples = radio_get_df_sample_amount();
            app_vars.pair_pending = 1;
            app_vars.pair_start_timestamp = timestamp;

            app_vars.tx1_done = 1;
            app_vars.tx1_done_timestamp = timestamp;
            app_vars.tx1_packet_sqn = app_vars.rxpk_buf[33];
            app_vars.tx1_num_samples = received_samples;
            if (received_samples <= NUM_SAMPLES) {
                memcpy(app_vars.tx1_sample_buffer,
                       app_vars.rx_sample_buffer,
                       received_samples * sizeof(uint32_t));
            }

            leds_error_toggle();
        }

        if (app_vars.rxpk_buf[34] == 2) {
            received_samples = radio_get_df_sample_amount();
            app_vars.pair_pending = 1;
            app_vars.pair_start_timestamp = timestamp;

            leds_debug_toggle();

            app_vars.tx2_done = 1;
            app_vars.tx2_done_timestamp = timestamp;
            app_vars.tx2_packet_sqn = app_vars.rxpk_buf[33];
            app_vars.tx2_num_samples = received_samples;
            if (received_samples <= NUM_SAMPLES) {
                memcpy(app_vars.tx2_sample_buffer,
                       app_vars.rx_sample_buffer,
                       received_samples * sizeof(uint32_t));
            }
        }
    }

    // Every reception uses a neutral staging buffer.  The completed IQ block
    // is copied to TX1/TX2 storage only after the packet ID is known.
    radio_set_df_sample_buffer(app_vars.rx_sample_buffer,NUM_SAMPLES);
    radio_rxEnable();
    radio_rxNow();

    // radio_rxEnable() calls leds_radio_on(); restore LED2 from our own state.
    if (app_vars.led2_match_state) {
        leds_radio_on();
    } else {
        leds_radio_off();
    }
}

void cb_uartTxDone(void) {
   app_vars.uart_lastTxByteIndex++;
   if (app_vars.uart_lastTxByteIndex < app_vars.uart_tx_len) {
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

   byte = uart_readByte();
   uart_writeByte(byte);

   return 0;
}
