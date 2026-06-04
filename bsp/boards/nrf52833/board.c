/**
\brief nRF52840-specific definition of the "board" bsp module.

\author Tengfei Chang <tengfei.chang@gmail.com>, July 2020.
*/

#include "nrf52833.h"
#include "board.h"
#include "nrf52833_bitfields.h"
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

    // POF warning is a software-visible low-voltage warning. Keep it disabled
    // for this low-power beacon; hardware brownout reset cannot be disabled.
    NRF_POWER->INTENCLR       = POWER_INTENCLR_POFWARN_Msk;
    NRF_POWER->EVENTS_POFWARN = (uint32_t)0;
    NRF_POWER->POFCON         =
        (POWER_POFCON_POF_Disabled << POWER_POFCON_POF_Pos);
    NVIC_ClearPendingIRQ(POWER_CLOCK_IRQn);

    uart_disable();
    i2c_disable();
    // initialize bsp modules
    // debugpins_init();
    // leds_init();
    // uart_init();
    radio_init();
    sctimer_init();
    // i2c_init();
    
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

void board_start_hfclk(void) {
    clocks_start();
}

void board_stop_hfclk(void) {
    NRF_CLOCK->TASKS_HFCLKSTOP = (uint32_t)1;
}

//=========================== private =========================================

void clocks_start( void ){

    uint32_t timeout;

    if (
        ((NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_STATE_Msk) != 0) &&
        ((NRF_CLOCK->HFCLKSTAT & CLOCK_HFCLKSTAT_SRC_Msk) ==
            (CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos))
    ) {
        return;
    }

    // Start HFCLK and wait for it to start.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    timeout = CLOCK_START_TIMEOUT;
    while ((NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) && (timeout > 0)) {
        timeout--;
    }
}
