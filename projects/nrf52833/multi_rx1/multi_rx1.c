/**
\brief Four-node IQ receiver and time-slotted collector reporter.

Build the same source with RX_NODE_ID set to 1, 2, 3, or 4.  Each node
captures TX1/TX2 IQ, waits for 100 ms of silence, then reports in its own
collector slot.  Complete IQ uses three radio fragments; incomplete rounds
use one zero-data placeholder packet.
*/

#include "stdint.h"
#include "string.h"
#include "board.h"
#include "radio.h"
#include "leds.h"
#include "radio_df.h"
#include "timer.h"

//=========================== defines =========================================

#ifndef RX_NODE_ID
#define RX_NODE_ID                 4
#endif

#if RX_NODE_ID < 1 || RX_NODE_ID > 4
#error "RX_NODE_ID must be 1, 2, 3, or 4"
#endif

#define CHANNEL                    17
#define TX_PACKET_MAX_LEN          128
#define NUM_SAMPLES                SAMPLE_MAXCNT
#define IQ_BYTES_PER_TX            (NUM_SAMPLES * 4)
#define IQ_BYTES_TOTAL             (IQ_BYTES_PER_TX * 2)

#define PAIR_TIMEOUT_TICKS         ((16000000/1000)*100)
#define TX1_TO_TX2_START_TICKS     ((16000000/1000)*5)
#define REPORT_SLOT_BASE_TICKS     ((16000000/1000)*150)
#define REPORT_SLOT_SPACING_TICKS  ((16000000/1000)*40)

// RX -> collector radio packet:
// [0] S0=0xA5, [1] payload length
// [2] magic=0xC1, [3] rx_id, [4] tx1_seq, [5] tx2_seq, [6] flags
// [7] fragment index, [8] fragment count, [9] data length
// [10] tx1 IQ count, [11] tx2 IQ count, [12..] IQ bytes
#define REPORT_S0                  0xA5
#define REPORT_MAGIC               0xC1
#define REPORT_HEADER_BYTES        10
#define REPORT_FRAGMENT_DATA       240
#define REPORT_FRAGMENT_COUNT      3
#define REPORT_FRAGMENT_GAP_TICKS  ((16000000/1000)*1)
#define REPORT_PACKET_MAX_LEN      (2 + REPORT_HEADER_BYTES + REPORT_FRAGMENT_DATA)

#define REPORT_FLAG_TX1            0x01
#define REPORT_FLAG_TX2            0x02
#define REPORT_FLAG_COMPLETE       0x04

typedef enum {
    APP_STATE_RX = 0,
    APP_STATE_REPORT_TX,
} app_state_t;

typedef struct {
    app_state_t       state;
    uint8_t           rxpk_buf[TX_PACKET_MAX_LEN];
    uint8_t           rxpk_len;
    int8_t            rxpk_rssi;
    uint8_t           rxpk_lqi;
    bool              rxpk_crc;

    uint32_t          tx1_samples[NUM_SAMPLES];
    uint32_t          tx2_samples[NUM_SAMPLES];
    uint32_t          rx_samples[NUM_SAMPLES];

    volatile bool     tx1_done;
    volatile bool     tx2_done;
    volatile uint8_t  tx1_seq;
    volatile uint8_t  tx2_seq;
    volatile uint16_t tx1_iq_count;
    volatile uint16_t tx2_iq_count;
    volatile uint32_t pair_last_timestamp;
    volatile uint32_t round_anchor_timestamp;
    volatile uint32_t frame_start_timestamp;
    volatile bool     pair_pending;

    uint8_t           report_packet[REPORT_PACKET_MAX_LEN];
    uint8_t           report_flags;
    uint8_t           report_fragment;
    uint8_t           report_packets_total;
    volatile bool     report_tx_done;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void cb_startFrame(PORT_TIMER_WIDTH timestamp);
void cb_endFrame(PORT_TIMER_WIDTH timestamp);
void clear_pair_state(void);
void send_report_fragment(void);

//=========================== main ============================================

int mote_main(void) {
    uint32_t now;
    uint32_t report_at;

    memset(&app_vars, 0, sizeof(app_vars));
    board_init();
    leds_init();
    timer_init();
    timer_start();

    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);
    radio_rfOn();
    radio_setFrequency(CHANNEL, FREQ_RX);
    radio_configure_direction_finding_manual_AoA();
    radio_set_df_sample_buffer(app_vars.rx_samples, NUM_SAMPLES);

    app_vars.state = APP_STATE_RX;
    radio_rxEnable();
    radio_rxNow();
    leds_all_off();

    while (1) {
        if (!app_vars.pair_pending) {
            continue;
        }

        timer_capture_now(0);
        now = timer_getCapturedValue(0);
        if ((now - app_vars.pair_last_timestamp) <= PAIR_TIMEOUT_TICKS) {
            continue;
        }

        app_vars.pair_pending = FALSE;
        app_vars.report_flags = 0;
        if (app_vars.tx1_done) {
            app_vars.report_flags |= REPORT_FLAG_TX1;
        }
        if (app_vars.tx2_done) {
            app_vars.report_flags |= REPORT_FLAG_TX2;
        }
        if (
            app_vars.tx1_done &&
            app_vars.tx2_done &&
            app_vars.tx1_seq == app_vars.tx2_seq &&
            app_vars.tx1_iq_count == NUM_SAMPLES &&
            app_vars.tx2_iq_count == NUM_SAMPLES
        ) {
            app_vars.report_flags |= REPORT_FLAG_COMPLETE;
            app_vars.report_packets_total = REPORT_FRAGMENT_COUNT;
            leds_radio_toggle();
        } else {
            // One short packet tells the collector exactly what was missing.
            app_vars.report_packets_total = 1;
        }

        report_at = app_vars.round_anchor_timestamp +
                    REPORT_SLOT_BASE_TICKS +
                    (RX_NODE_ID - 1) * REPORT_SLOT_SPACING_TICKS;
        do {
            timer_capture_now(0);
            now = timer_getCapturedValue(0);
        } while ((int32_t)(now - report_at) < 0);

        app_vars.report_fragment = 0;
        app_vars.report_tx_done = FALSE;
        app_vars.state = APP_STATE_REPORT_TX;
        send_report_fragment();

        while (!app_vars.report_tx_done) {
            board_sleep();
        }

        clear_pair_state();
    }
}

//=========================== private =========================================

void clear_pair_state(void) {
    app_vars.tx1_done = FALSE;
    app_vars.tx2_done = FALSE;
    app_vars.tx1_seq = 0;
    app_vars.tx2_seq = 0;
    app_vars.tx1_iq_count = 0;
    app_vars.tx2_iq_count = 0;
    app_vars.pair_pending = FALSE;
    memset(app_vars.tx1_samples, 0, sizeof(app_vars.tx1_samples));
    memset(app_vars.tx2_samples, 0, sizeof(app_vars.tx2_samples));
    memset(app_vars.rx_samples, 0, sizeof(app_vars.rx_samples));
    radio_set_df_sample_buffer(app_vars.rx_samples, NUM_SAMPLES);
}

void send_report_fragment(void) {
    uint16_t offset;
    uint16_t data_len;
    uint16_t i;
    uint16_t byte_index;
    uint16_t sample_index;
    uint8_t byte_in_sample;
    uint32_t sample;

    memset(app_vars.report_packet, 0, sizeof(app_vars.report_packet));
    data_len = 0;

    if (app_vars.report_flags & REPORT_FLAG_COMPLETE) {
        offset = app_vars.report_fragment * REPORT_FRAGMENT_DATA;
        data_len = IQ_BYTES_TOTAL - offset;
        if (data_len > REPORT_FRAGMENT_DATA) {
            data_len = REPORT_FRAGMENT_DATA;
        }

        for (i = 0; i < data_len; i++) {
            byte_index = offset + i;
            if (byte_index < IQ_BYTES_PER_TX) {
                sample_index = byte_index / 4;
                byte_in_sample = byte_index % 4;
                sample = app_vars.tx1_samples[sample_index];
            } else {
                byte_index -= IQ_BYTES_PER_TX;
                sample_index = byte_index / 4;
                byte_in_sample = byte_index % 4;
                sample = app_vars.tx2_samples[sample_index];
            }
            app_vars.report_packet[12 + i] =
                (sample >> (24 - 8 * byte_in_sample)) & 0xff;
        }
    }

    app_vars.report_packet[0]  = REPORT_S0;
    app_vars.report_packet[1]  = REPORT_HEADER_BYTES + data_len;
    app_vars.report_packet[2]  = REPORT_MAGIC;
    app_vars.report_packet[3]  = RX_NODE_ID;
    app_vars.report_packet[4]  = app_vars.tx1_seq;
    app_vars.report_packet[5]  = app_vars.tx2_seq;
    app_vars.report_packet[6]  = app_vars.report_flags;
    app_vars.report_packet[7]  = app_vars.report_fragment;
    app_vars.report_packet[8]  =
        (app_vars.report_flags & REPORT_FLAG_COMPLETE) ?
        REPORT_FRAGMENT_COUNT : 0;
    app_vars.report_packet[9]  = data_len;
    app_vars.report_packet[10] = app_vars.tx1_iq_count;
    app_vars.report_packet[11] = app_vars.tx2_iq_count;

    radio_rfOff();
    radio_setFrequency(CHANNEL, FREQ_TX);
    radio_loadPacket(app_vars.report_packet, 12 + data_len);
    radio_txEnable();
    radio_txNow();
}

//=========================== callbacks =======================================

void cb_startFrame(PORT_TIMER_WIDTH timestamp) {
    if (app_vars.state == APP_STATE_RX) {
        app_vars.frame_start_timestamp = timestamp;
    }
}

void cb_endFrame(PORT_TIMER_WIDTH timestamp) {
    uint16_t samples;
    uint32_t now;
    bool target;

    if (app_vars.state == APP_STATE_REPORT_TX) {
        app_vars.report_fragment++;
        if (app_vars.report_fragment < app_vars.report_packets_total) {
            // Give the collector enough time to leave PHYEND and re-arm RX.
            do {
                timer_capture_now(2);
                now = timer_getCapturedValue(2);
            } while ((now - timestamp) < REPORT_FRAGMENT_GAP_TICKS);
            send_report_fragment();
        } else {
            radio_rfOn();
            radio_setFrequency(CHANNEL, FREQ_RX);
            radio_configure_direction_finding_manual_AoA();
            radio_set_df_sample_buffer(app_vars.rx_samples, NUM_SAMPLES);
            app_vars.state = APP_STATE_RX;
            radio_rxEnable();
            radio_rxNow();
            app_vars.report_tx_done = TRUE;
        }
        return;
    }

    memset(app_vars.rxpk_buf, 0, sizeof(app_vars.rxpk_buf));
    app_vars.rxpk_len = sizeof(app_vars.rxpk_buf);
    radio_getReceivedFrame(
        app_vars.rxpk_buf,
        &app_vars.rxpk_len,
        sizeof(app_vars.rxpk_buf),
        &app_vars.rxpk_rssi,
        &app_vars.rxpk_lqi,
        &app_vars.rxpk_crc
    );

    target =
        app_vars.rxpk_crc &&
        app_vars.rxpk_len > 34 &&
        app_vars.rxpk_len <= TX_PACKET_MAX_LEN &&
        app_vars.rxpk_buf[0] == 0x42 &&
        app_vars.rxpk_buf[1] == 0x21 &&
        (app_vars.rxpk_buf[34] == 1 || app_vars.rxpk_buf[34] == 2);

    if (target) {
        samples = radio_get_df_sample_amount();
        app_vars.pair_pending = TRUE;
        app_vars.pair_last_timestamp = timestamp;

        if (app_vars.rxpk_buf[34] == 1) {
            app_vars.round_anchor_timestamp = app_vars.frame_start_timestamp;
            app_vars.tx1_done = TRUE;
            app_vars.tx1_seq = app_vars.rxpk_buf[33];
            app_vars.tx1_iq_count = samples;
            if (samples <= NUM_SAMPLES) {
                memcpy(app_vars.tx1_samples,
                       app_vars.rx_samples,
                       samples * sizeof(uint32_t));
            }
            leds_error_toggle();
        } else {
            if (!app_vars.tx1_done) {
                app_vars.round_anchor_timestamp =
                    app_vars.frame_start_timestamp - TX1_TO_TX2_START_TICKS;
            }
            app_vars.tx2_done = TRUE;
            app_vars.tx2_seq = app_vars.rxpk_buf[33];
            app_vars.tx2_iq_count = samples;
            if (samples <= NUM_SAMPLES) {
                memcpy(app_vars.tx2_samples,
                       app_vars.rx_samples,
                       samples * sizeof(uint32_t));
            }
            leds_debug_toggle();
        }
    }

    radio_set_df_sample_buffer(app_vars.rx_samples, NUM_SAMPLES);
    radio_rxEnable();
    radio_rxNow();
}
