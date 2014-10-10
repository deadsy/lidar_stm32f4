//-----------------------------------------------------------------------------
/*

PWM Driver

*/
//-----------------------------------------------------------------------------

#include <string.h>

#include "timers.h"
#include "pwm.h"

//-----------------------------------------------------------------------------

#define PWM_TIMER_PERIOD 1000

//-----------------------------------------------------------------------------

#define NUM_PWMS 1
static PWM_t pwms[NUM_PWMS];

//-----------------------------------------------------------------------------

void pwm_hw_init(PWM_t *pwm) {

    TIM_TypeDef* tim = pwm->tim;

    // enable the peripheral clock
    tim_enable_clock(tim);

    // up counter, edge aligned mode, arr is buffered
    tim->CR1 = TIM_CR1_ARPE;
    tim->CR2 = 0;
    // slave mode control register (not used)
    tim->SMCR = 0;
    // disable and clear interrupts
    tim->DIER = 0;
    tim->SR = 0;
    // setup the output control mode
    tim->CCMR1 = (6 << 12 /*OC2 = PWM mode 1*/) | (1 << 11 /*OC2 preload enable*/);
    tim->CCMR2 = 0;
    // disable any output (for now)
    tim->CCER = 0;
    // setup the counter, reload value and prescalar
    tim->CNT = 0;
    tim->PSC = ((SystemCoreClock / 2) /(PWM_TIMER_PERIOD * pwm->freq)) - 1;
    tim->ARR = PWM_TIMER_PERIOD - 1;
    // setup the output compare values
    tim->CCR1 = 0;
    tim->CCR2 = 0;
    tim->CCR3 = 0;
    tim->CCR4 = 0;
    // dma is not used
    tim->DCR = 0;
    tim->DMAR = 0;

    // generate update event to load registers
    tim->EGR = TIM_EGR_UG;
}

//-----------------------------------------------------------------------------
// clamp float value

static float clamp_f(float x, float lo, float hi) {
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

//-----------------------------------------------------------------------------

float pwm_get_period(PWM_t *pwm) {
    return pwm->duty_cycle;
}

void pwm_set(PWM_t *pwm, float x) {
    pwm->duty_cycle = clamp_f(x, 0.0, 1.0);
    pwm->tim->CCR1 = pwm->duty_cycle * PWM_TIMER_PERIOD;
}

void pwm_delta(PWM_t *pwm, float x) {
    pwm_set(pwm, pwm->duty_cycle + x);
}

//-----------------------------------------------------------------------------

PWM_t *pwm_init(unsigned int idx, uint32_t freq) {

    if (idx >= NUM_PWMS) {
        return 0;
    }

    PWM_t *pwm = &pwms[idx];
    memset(pwm, 0, sizeof(PWM_t));
    pwm->freq = freq;

    if (idx == 0) {
        pwm->tim = TIM2;
    } else {
        return 0;
    }

    pwm_hw_init(pwm);
    return pwm;
}

//-----------------------------------------------------------------------------
