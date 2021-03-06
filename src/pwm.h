//-----------------------------------------------------------------------------
/*

PWM Driver

*/
//-----------------------------------------------------------------------------

#ifndef PWM_H
#define PWM_H

//-----------------------------------------------------------------------------

#include "stm32f4xx_hal.h"

//-----------------------------------------------------------------------------

typedef struct pwm_driver {
    TIM_TypeDef *tim;
    uint32_t freq;
    float duty_cycle;

} PWM_t;

//-----------------------------------------------------------------------------

PWM_t *pwm_init(unsigned int idx, uint32_t freq);
void pwm_set(PWM_t *pwm, float x);
float pwm_get_period(PWM_t *pwm);
void pwm_delta(PWM_t *pwm, float x);

//-----------------------------------------------------------------------------

#endif // PWM_H

//-----------------------------------------------------------------------------
