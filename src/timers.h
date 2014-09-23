//-----------------------------------------------------------------------------
/*

Timer Functions

The main stepper routines use timers and ISRs for pulse generation.
This file has the board specific routines to allow this.

*/
//-----------------------------------------------------------------------------

#ifndef TIMERS_H
#define TIMERS_H

//-----------------------------------------------------------------------------

#include "stm32f4xx_hal.h"

//-----------------------------------------------------------------------------

// utility functions
void tim_enable_clock(TIM_TypeDef *tim);
void tim_enable_interrupt(TIM_TypeDef *tim, uint32_t pre, uint32_t sub);

// cdc interface
void cdc_timer_start(void);
void cdc_timer_isr(void);

// general
void timers_init(void);

//-----------------------------------------------------------------------------

#endif // TIMERS_H

//-----------------------------------------------------------------------------