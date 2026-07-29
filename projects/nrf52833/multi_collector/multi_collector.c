/**
\brief Collector for four time-slotted multi_rx1 IQ receiver nodes.

The collector reassembles three-fragment IQ reports (or short placeholders)
and forwards one simple binary UART frame per RX node.
*/

#include "stdint.h"
#include "string.h"
#include "board.h"
#include "radio.h"
#include "leds.h"
#include "uart.h"
#include "timer.h"

//=========================== defines =========================================

#define CHANNEL                    17
#define RX_NODE_COUNT              4
#define IQ_BYTES_TOTAL             704
#define REPORT_PACKET_MAX_LEN      252
#define REPORT_FRAGMENT_DATA       240
#define REPORT_FRAGMENT_COUNT      3
#define REPORT_S0                  0xA5
#define REPORT_MAGIC               0xC1
#define REPORT_FLAG_TX1            0x01
#define REPORT_FLAG_TX2            0x02
#define REPORT_FLAG_COMPLETE       0x04
#define COLLECTION_TIMEOUT_TICKS   ((16000000/1000)*180)

// Collector -> PC UART frame:
// 55 AA | version | rx_id | tx1_seq | tx2_seq | flags |
// tx1_iq_count | tx2_iq_count | data_len_be16 | data | xor_checksum
#define UART_VERSION               1
#define UART_HEADER_LEN            11
#define UART_FRAME_MAX_LEN         (UART_HEADER_LEN + IQ_BYTES_TOTAL + 1)

typedef struct {
    uint8_t           radio_packet[REPORT_PACKET_MAX_LEN];
    uint8_t           radio_len;
    int8_t            radio_rssi;
    uint8_t           radio_lqi;
    bool              radio_crc;

    uint8_t           iq[RX_NODE_COUNT][IQ_BYTES_TOTAL];
    uint8_t           tx1_seq[RX_NODE_COUNT];
    uint8_t           tx2_seq[RX_NODE_COUNT];
    uint8_t           flags[RX_NODE_COUNT];
    uint8_t           tx1_iq_count[RX_NODE_COUNT];
    uint8_t           tx2_iq_count[RX_NODE_COUNT];
    uint8_t           fragment_mask[RX_NODE_COUNT];
    volatile uint8_t  ready[RX_NODE_COUNT];
    volatile uint8_t  sent[RX_NODE_COUNT];

    volatile uint8_t  collection_active;
    volatile uint8_t  collection_closed;
    volatile uint32_t collection_start;
    volatile uint8_t  round_seq;

    uint8_t           uart_frame[UART_FRAME_MAX_LEN];
    volatile uint16_t uart_len;
    volatile uint16_t uart_index;
    volatile uint8_t  uart_busy;
} app_vars_t;

app_vars_t app_vars;

//=========================== prototypes ======================================

void cb_startFrame(PORT_TIMER_WIDTH timestamp);
void cb_endFrame(PORT_TIMER_WIDTH timestamp);
void cb_uartTxDone(void);
uint8_t cb_uartRx(void);
void reset_collection(void);
void start_uart_node(uint8_t node);

//=========================== main ============================================

int mote_main(void) {
    uint8_t i;
    uint8_t all_ready;
    uint8_t all_sent;
    uint32_t now;

    memset(&app_vars, 0, sizeof(app_vars));
    board_init();
    leds_init();
    timer_init();
    timer_start();
    uart_init();
    uart_setCallbacks(cb_uartTxDone, cb_uartRx);
    uart_enableInterrupts();

    radio_setStartFrameCb(cb_startFrame);
    radio_setEndFrameCb(cb_endFrame);
    radio_rfOn();
    radio_setFrequency(CHANNEL, FREQ_RX);
    radio_rxEnable();
    radio_rxNow();
    leds_all_off();

    while (1) {
        if (!app_vars.collection_active) {
            continue;
        }

        all_ready = TRUE;
        for (i = 0; i < RX_NODE_COUNT; i++) {
            if (!app_vars.ready[i]) {
                all_ready = FALSE;
            }
        }

        timer_capture_now(0);
        now = timer_getCapturedValue(0);
        if (
            all_ready ||
            (now - app_vars.collection_start) > COLLECTION_TIMEOUT_TICKS
        ) {
            app_vars.collection_closed = TRUE;
            for (i = 0; i < RX_NODE_COUNT; i++) {
                if (!app_vars.ready[i]) {
                    // No report from this RX node: both TX1 and TX2 are missing.
                    app_vars.tx1_seq[i] = app_vars.round_seq;
                    app_vars.tx2_seq[i] = app_vars.round_seq;
                    app_vars.flags[i] = 0;
                    app_vars.tx1_iq_count[i] = 0;
                    app_vars.tx2_iq_count[i] = 0;
                    app_vars.ready[i] = TRUE;
                }
            }
        }

        if (!app_vars.uart_busy) {
            for (i = 0; i < RX_NODE_COUNT; i++) {
                if (app_vars.ready[i] && !app_vars.sent[i]) {
                    start_uart_node(i);
                    break;
                }
            }
        }

        if (app_vars.collection_closed && !app_vars.uart_busy) {
            all_sent = TRUE;
            for (i = 0; i < RX_NODE_COUNT; i++) {
                if (!app_vars.sent[i]) {
                    all_sent = FALSE;
                }
            }
            if (all_sent) {
                reset_collection();
            }
        }
    }
}

//=========================== private =========================================

void reset_collection(void) {
    memset(app_vars.iq, 0, sizeof(app_vars.iq));
    memset(app_vars.tx1_seq, 0, sizeof(app_vars.tx1_seq));
    memset(app_vars.tx2_seq, 0, sizeof(app_vars.tx2_seq));
    memset(app_vars.flags, 0, sizeof(app_vars.flags));
    memset(app_vars.tx1_iq_count, 0, sizeof(app_vars.tx1_iq_count));
    memset(app_vars.tx2_iq_count, 0, sizeof(app_vars.tx2_iq_count));
    memset(app_vars.fragment_mask, 0, sizeof(app_vars.fragment_mask));
    memset((void*)app_vars.ready, 0, sizeof(app_vars.ready));
    memset((void*)app_vars.sent, 0, sizeof(app_vars.sent));
    app_vars.collection_active = FALSE;
    app_vars.collection_closed = FALSE;
}

void start_uart_node(uint8_t node) {
    uint16_t i;
    uint16_t data_len;
    uint8_t checksum;

    data_len =
        (app_vars.flags[node] & REPORT_FLAG_COMPLETE) ?
        IQ_BYTES_TOTAL : 0;

    app_vars.uart_frame[0]  = 0x55;
    app_vars.uart_frame[1]  = 0xAA;
    app_vars.uart_frame[2]  = UART_VERSION;
    app_vars.uart_frame[3]  = node + 1;
    app_vars.uart_frame[4]  = app_vars.tx1_seq[node];
    app_vars.uart_frame[5]  = app_vars.tx2_seq[node];
    app_vars.uart_frame[6]  = app_vars.flags[node];
    app_vars.uart_frame[7]  = app_vars.tx1_iq_count[node];
    app_vars.uart_frame[8]  = app_vars.tx2_iq_count[node];
    app_vars.uart_frame[9]  = (data_len >> 8) & 0xff;
    app_vars.uart_frame[10] = data_len & 0xff;
    if (data_len) {
        memcpy(&app_vars.uart_frame[UART_HEADER_LEN],
               app_vars.iq[node],
               data_len);
    }

    checksum = 0;
    for (i = 2; i < UART_HEADER_LEN + data_len; i++) {
        checksum ^= app_vars.uart_frame[i];
    }
    app_vars.uart_frame[UART_HEADER_LEN + data_len] = checksum;
    app_vars.uart_len = UART_HEADER_LEN + data_len + 1;
    app_vars.uart_index = 0;
    app_vars.uart_busy = TRUE;
    app_vars.sent[node] = TRUE;
    uart_writeByte(app_vars.uart_frame[0]);
}

//=========================== callbacks =======================================

void cb_startFrame(PORT_TIMER_WIDTH timestamp) {
    (void)timestamp;
}

void cb_endFrame(PORT_TIMER_WIDTH timestamp) {
    uint8_t node;
    uint8_t fragment;
    uint8_t fragment_count;
    uint8_t data_len;
    uint16_t offset;
    bool valid;

    memset(app_vars.radio_packet, 0, sizeof(app_vars.radio_packet));
    app_vars.radio_len = sizeof(app_vars.radio_packet);
    radio_getReceivedFrame(
        app_vars.radio_packet,
        &app_vars.radio_len,
        sizeof(app_vars.radio_packet),
        &app_vars.radio_rssi,
        &app_vars.radio_lqi,
        &app_vars.radio_crc
    );

    data_len = app_vars.radio_packet[9];
    valid =
        app_vars.radio_crc &&
        app_vars.radio_len >= 12 &&
        app_vars.radio_packet[0] == REPORT_S0 &&
        app_vars.radio_packet[2] == REPORT_MAGIC &&
        app_vars.radio_packet[3] >= 1 &&
        app_vars.radio_packet[3] <= RX_NODE_COUNT &&
        app_vars.radio_len == (uint8_t)(12 + data_len);

    if (valid) {
        node = app_vars.radio_packet[3] - 1;
        fragment = app_vars.radio_packet[7];
        fragment_count = app_vars.radio_packet[8];
        offset = fragment * REPORT_FRAGMENT_DATA;

        if (!app_vars.collection_active) {
            reset_collection();
            app_vars.collection_active = TRUE;
            app_vars.collection_start = timestamp;
            app_vars.round_seq =
                (app_vars.radio_packet[6] & REPORT_FLAG_TX1) ?
                app_vars.radio_packet[4] : app_vars.radio_packet[5];
        }

        if (fragment == 0) {
            app_vars.fragment_mask[node] = 0;
            app_vars.ready[node] = FALSE;
            app_vars.sent[node] = FALSE;
            memset(app_vars.iq[node], 0, IQ_BYTES_TOTAL);
        }

        app_vars.tx1_seq[node] = app_vars.radio_packet[4];
        app_vars.tx2_seq[node] = app_vars.radio_packet[5];
        app_vars.flags[node] = app_vars.radio_packet[6];
        app_vars.tx1_iq_count[node] = app_vars.radio_packet[10];
        app_vars.tx2_iq_count[node] = app_vars.radio_packet[11];

        if (fragment_count == 0 && data_len == 0) {
            app_vars.ready[node] = TRUE;
        } else if (
            fragment_count == REPORT_FRAGMENT_COUNT &&
            fragment < REPORT_FRAGMENT_COUNT &&
            data_len <= REPORT_FRAGMENT_DATA &&
            offset + data_len <= IQ_BYTES_TOTAL
        ) {
            memcpy(&app_vars.iq[node][offset],
                   &app_vars.radio_packet[12],
                   data_len);
            app_vars.fragment_mask[node] |= 1 << fragment;
            if (app_vars.fragment_mask[node] == 0x07) {
                app_vars.ready[node] = TRUE;
            }
        }
        leds_error_toggle();
    }

    radio_rxEnable();
    radio_rxNow();
}

void cb_uartTxDone(void) {
    app_vars.uart_index++;
    if (app_vars.uart_index < app_vars.uart_len) {
        uart_writeByte(app_vars.uart_frame[app_vars.uart_index]);
    } else {
        app_vars.uart_busy = FALSE;
        leds_debug_toggle();
    }
}

uint8_t cb_uartRx(void) {
    (void)uart_readByte();
    return 0;
}
