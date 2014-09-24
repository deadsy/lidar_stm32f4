//-----------------------------------------------------------------------------
/*

Timer Functions

Setup the hardware timers for:

* The generation of periodic callbacks to the cdc interface

*/
//-----------------------------------------------------------------------------

#include "timers.h"

//-----------------------------------------------------------------------------

// enable the peripheral clock for the timers
void tim_enable_clock(TIM_TypeDef *tim)
{
    if (tim == TIM2) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    } else if (tim == TIM3) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    } else if (tim == TIM4) {
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    }
}

// enable timer interrupts
// set the preempt and sub priority
void tim_enable_interrupt(TIM_TypeDef *tim, uint32_t pre, uint32_t sub)
{
    uint32_t irq;

    if (tim == TIM2) {
        irq = TIM2_IRQn;
    } else if (tim == TIM3) {
        irq = TIM3_IRQn;
    } else if (tim == TIM4) {
        irq = TIM4_IRQn;
    } else {
        return;
    }

    HAL_NVIC_SetPriority(irq, pre, sub);
    NVIC_EnableIRQ(irq);
}

//-----------------------------------------------------------------------------
// Generate a periodic callback to the CDC interface

#define CDC_TIMER TIM3
#define CDC_TIMER_HZ 200
#define CDC_TIMER_PERIOD 5000

static void cdc_timer_init(void)
{
    TIM_TypeDef* const TIMx = CDC_TIMER;

    // enable the peripheral clock
    tim_enable_clock(TIMx);

    // up counter, edge aligned mode, arr is buffered
    TIMx->CR1 = TIM_CR1_ARPE;
    TIMx->CR2 = 0;
    // slave mode control register (not used)
    TIMx->SMCR = 0;
    // disable and clear interrupts
    TIMx->DIER = 0;
    TIMx->SR = 0;
    // no output/input control
    TIMx->CCMR1 = 0;
    TIMx->CCMR2 = 0;
    TIMx->CCER = 0;
    // setup the counter, reload value and prescalar
    TIMx->CNT = 0;
    TIMx->PSC = ((SystemCoreClock / 2) /(CDC_TIMER_PERIOD * CDC_TIMER_HZ)) - 1;
    TIMx->ARR = CDC_TIMER_PERIOD - 1;
    // no output compare values
    TIMx->CCR1 = 0;
    TIMx->CCR2 = 0;
    TIMx->CCR3 = 0;
    TIMx->CCR4 = 0;
    // dma is not used
    TIMx->DCR = 0;
    TIMx->DMAR = 0;

    // generate update event to load registers
    TIMx->EGR = TIM_EGR_UG;

    // enable the interrupt - run this at low priority.
    tim_enable_interrupt(TIMx, 5, 0);
}

void cdc_timer_start(void)
{
    TIM_TypeDef* const TIMx = CDC_TIMER;

    // enable update interrupts
    TIMx->DIER |= TIM_DIER_UIE;
    // turn on the timer
    TIMx->CR1 |= TIM_CR1_CEN;
}

void TIM3_IRQHandler(void)
{
    TIM_TypeDef* const TIMx = CDC_TIMER;

    if ((TIMx->SR & TIM_SR_UIF) && (TIMx->DIER & TIM_DIER_UIE)) {
        TIMx->SR &= ~TIM_SR_UIF;
        //cdc_timer_isr();
    }
}

//-----------------------------------------------------------------------------

void timers_init(void)
{
    cdc_timer_init();
}

//-----------------------------------------------------------------------------
