/**
\brief Insect robot PWM test project for nRF52833.

PWM output mapping:
   pwm1 -> P0.28
   pwm2 -> P0.29

LED1 is used as a debug heartbeat when duty cycles are updated.
*/

#include "stdint.h"
#include "board.h"
#include "leds.h"
#include "nrf52833.h"
#include "nrf52833_bitfields.h"

//=========================== defines =========================================

#define NRF_GPIO_PIN_MAP(port, pin) (((port) << 5) | ((pin) & 0x1f))

#define INSECT_PWM1_PIN          NRF_GPIO_PIN_MAP(0,28)
#define INSECT_PWM2_PIN          NRF_GPIO_PIN_MAP(0,29)

#define INSECT_PWM_COUNTERTOP    1000
#define INSECT_PWM_STEP          25
#define INSECT_PWM_DELAY         0x4ffff

//=========================== variables =======================================

static volatile uint16_t pwm_sequence[4] __attribute__((aligned(4)));

volatile uint16_t g_pwm1_duty_permille = 300;
volatile uint16_t g_pwm2_duty_permille = 700;
volatile uint8_t  g_pwm_auto_sweep     = 1;
volatile uint32_t g_pwm_update_count   = 0;

//=========================== prototypes ======================================

static void     insect_pwm_init(void);
static void     insect_pwm_gpio_cfg_output(uint32_t pin_number);
static uint16_t insect_pwm_clamp(uint16_t duty_permille);
static void     insect_delay(void);

void insect_robot_set_pwm_permille(uint16_t pwm1_duty_permille,
                                   uint16_t pwm2_duty_permille);

//=========================== main ============================================

int mote_main(void) {
    uint16_t sweep_duty;
    int16_t  sweep_step;

    sweep_duty = 0;
    sweep_step = INSECT_PWM_STEP;

    board_init();
    insect_pwm_init();

    while (1) {
        if (g_pwm_auto_sweep!=0) {
            insect_robot_set_pwm_permille(
                sweep_duty,
                INSECT_PWM_COUNTERTOP-sweep_duty
            );

            if (sweep_duty>=INSECT_PWM_COUNTERTOP) {
                sweep_step = -INSECT_PWM_STEP;
            } else if (sweep_duty==0) {
                sweep_step = INSECT_PWM_STEP;
            }

            sweep_duty = (uint16_t)((int16_t)sweep_duty+sweep_step);
        } else {
            insect_robot_set_pwm_permille(
                g_pwm1_duty_permille,
                g_pwm2_duty_permille
            );
        }

        insect_delay();
    }
}

//=========================== public ==========================================

void insect_robot_set_pwm_permille(uint16_t pwm1_duty_permille,
                                   uint16_t pwm2_duty_permille) {

    pwm1_duty_permille = insect_pwm_clamp(pwm1_duty_permille);
    pwm2_duty_permille = insect_pwm_clamp(pwm2_duty_permille);

    g_pwm1_duty_permille = pwm1_duty_permille;
    g_pwm2_duty_permille = pwm2_duty_permille;

    pwm_sequence[0] = pwm1_duty_permille;
    pwm_sequence[1] = pwm2_duty_permille;
    pwm_sequence[2] = 0;
    pwm_sequence[3] = 0;

    NRF_PWM0->TASKS_SEQSTART[0] =
        (PWM_TASKS_SEQSTART_TASKS_SEQSTART_Trigger
            << PWM_TASKS_SEQSTART_TASKS_SEQSTART_Pos);

    g_pwm_update_count++;
    leds_error_toggle();
}

//=========================== private =========================================

static void insect_pwm_init(void) {

    insect_pwm_gpio_cfg_output(INSECT_PWM1_PIN);
    insect_pwm_gpio_cfg_output(INSECT_PWM2_PIN);

    NRF_PWM0->TASKS_STOP =
        (PWM_TASKS_STOP_TASKS_STOP_Trigger
            << PWM_TASKS_STOP_TASKS_STOP_Pos);
    NRF_PWM0->ENABLE =
        (PWM_ENABLE_ENABLE_Disabled
            << PWM_ENABLE_ENABLE_Pos);

    NRF_PWM0->PSEL.OUT[0] = INSECT_PWM1_PIN;
    NRF_PWM0->PSEL.OUT[1] = INSECT_PWM2_PIN;
    NRF_PWM0->PSEL.OUT[2] =
        (PWM_PSEL_OUT_CONNECT_Disconnected
            << PWM_PSEL_OUT_CONNECT_Pos);
    NRF_PWM0->PSEL.OUT[3] =
        (PWM_PSEL_OUT_CONNECT_Disconnected
            << PWM_PSEL_OUT_CONNECT_Pos);

    NRF_PWM0->MODE =
        (PWM_MODE_UPDOWN_Up
            << PWM_MODE_UPDOWN_Pos);
    NRF_PWM0->COUNTERTOP = INSECT_PWM_COUNTERTOP;
    NRF_PWM0->PRESCALER =
        (PWM_PRESCALER_PRESCALER_DIV_16
            << PWM_PRESCALER_PRESCALER_Pos);
    NRF_PWM0->DECODER =
        (PWM_DECODER_LOAD_Individual
            << PWM_DECODER_LOAD_Pos) |
        (PWM_DECODER_MODE_RefreshCount
            << PWM_DECODER_MODE_Pos);
    NRF_PWM0->LOOP = 1;

    NRF_PWM0->SEQ[0].PTR = (uint32_t)(uintptr_t)&pwm_sequence[0];
    NRF_PWM0->SEQ[0].CNT = 4;
    NRF_PWM0->SEQ[0].REFRESH =
        (PWM_SEQ_REFRESH_CNT_Continuous
            << PWM_SEQ_REFRESH_CNT_Pos);
    NRF_PWM0->SEQ[0].ENDDELAY = 0;

    NRF_PWM0->SEQ[1].PTR = 0;
    NRF_PWM0->SEQ[1].CNT =
        (PWM_SEQ_CNT_CNT_Disabled
            << PWM_SEQ_CNT_CNT_Pos);
    NRF_PWM0->SEQ[1].REFRESH = 0;
    NRF_PWM0->SEQ[1].ENDDELAY = 0;

    NRF_PWM0->SHORTS =
        PWM_SHORTS_LOOPSDONE_SEQSTART0_Msk;

    NRF_PWM0->ENABLE =
        (PWM_ENABLE_ENABLE_Enabled
            << PWM_ENABLE_ENABLE_Pos);

    insect_robot_set_pwm_permille(
        g_pwm1_duty_permille,
        g_pwm2_duty_permille
    );
}

static void insect_pwm_gpio_cfg_output(uint32_t pin_number) {
    NRF_GPIO_Type *gpio_port;
    uint32_t       gpio_pin;

    gpio_pin = pin_number & 0x1f;
    if (pin_number<32) {
        gpio_port = NRF_P0;
    } else {
        gpio_port = NRF_P1;
    }

    gpio_port->PIN_CNF[gpio_pin] =
          ((uint32_t)GPIO_PIN_CNF_DIR_Output
            << GPIO_PIN_CNF_DIR_Pos)
        | ((uint32_t)GPIO_PIN_CNF_INPUT_Disconnect
            << GPIO_PIN_CNF_INPUT_Pos)
        | ((uint32_t)GPIO_PIN_CNF_PULL_Disabled
            << GPIO_PIN_CNF_PULL_Pos)
        | ((uint32_t)GPIO_PIN_CNF_DRIVE_S0S1
            << GPIO_PIN_CNF_DRIVE_Pos)
        | ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled
            << GPIO_PIN_CNF_SENSE_Pos);
}

static uint16_t insect_pwm_clamp(uint16_t duty_permille) {
    if (duty_permille>INSECT_PWM_COUNTERTOP) {
        return INSECT_PWM_COUNTERTOP;
    }

    return duty_permille;
}

static void insect_delay(void) {
    volatile uint32_t delay;

    for (delay=INSECT_PWM_DELAY; delay>0; delay--);
}
