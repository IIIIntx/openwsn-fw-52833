/**
\brief nRF52840-specific definition of the "board" bsp module.

\author Tengfei Chang <tengfei.chang@gmail.com>, July 2020.
*/

#include "nrf52833.h"
#include "board.h"
// bsp modules
#include "debugpins.h"
#include "leds.h"
#include "uart.h"
#include "sctimer.h"
#include "radio.h"
#include "i2c.h"

//=========================== variables =======================================

#define CLOCK_START_TIMEOUT 0x00ffffff

//=========================== prototypes ======================================

void clocks_start(void);

//=========================== main ============================================

extern int mote_main(void);

int main(void) {
   return mote_main();
}

//=========================== public ==========================================

void board_init(void) {

    clocks_start();

    // initialize bsp modules
    debugpins_init();
    leds_init();
    uart_init();
    radio_init();
    sctimer_init();
    i2c_init();
    
}

void board_sleep(void) {
    NRF_POWER->TASKS_LOWPWR    = (uint32_t)1;

    __SEV();
    __WFE();
    __WFE();
}

void board_reset(void) {
    // todo

    NVIC_SystemReset();
}

//=========================== private =========================================

void clocks_start( void ){

    uint32_t timeout;

    // Start HFCLK and wait for it to start.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    timeout = CLOCK_START_TIMEOUT;
    while ((NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) && (timeout > 0)) {
        timeout--;
    }
}
