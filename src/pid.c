//-----------------------------------------------------------------------------
/*

PID - Proportional, Integral, Derivative Controller

*/
//-----------------------------------------------------------------------------

#include <string.h>
#include <stdio.h>

#include "pid.h"

//-----------------------------------------------------------------------------
// PID - ideal parallel form

// Note: It is assumed that this function gets called at a constant periodic
// interval and that the fixed time associated with the integral/derivative
// gets absorbed into the ki/kd constants respectively.

void pid(PID_CTRL *p, float error, float *out, int debug)
{
    // if error exceeds maximum limits set output to 0
    if (p->e_limit != 0.0) {
        if ((error > p->e_limit) || (error < -p->e_limit)) {
            *out = 0.0;
            return;
        }
    }

    float p_term, i_term, d_term;
    p_term = i_term = d_term = 0.0;

    // proportional
    if (p->kp != 0.0) {
        p_term = error * p->kp;
    }

    // integral
    if (p->ki != 0.0) {
        p->isum += error;

        // clamp the integral sum
        if (p->i_limit != 0.0) {
            if (p->isum > p->i_limit) {
                p->isum = p->i_limit;
            }
            if (p->isum < -p->i_limit) {
                p->isum = -p->i_limit;
            }
        }

        i_term = p->ki * p->isum;
    }

    // derivative
    if (p->kd != 0.0) {
        d_term = (error - p->prev) * p->kd;
        p->prev = error;
    }

    if (debug) {
        printf("pid,%.2e,%.2e,%.2e:", p_term, i_term, d_term);
    }

    *out = p_term + i_term + d_term;
}

//-----------------------------------------------------------------------------

// Initialise kp, ki, kd directly
void pid_init1(PID_CTRL *p, float kp, float ki, float kd)
{
    memset(p, 0, sizeof(PID_CTRL));
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
}

// Initialise kp, ki, kd from time constants
void pid_init2(PID_CTRL *p, float kp, float ti, float td, float dt)
{
    memset(p, 0, sizeof(PID_CTRL));
    p->kp = kp;
    if (ti != 0.0) {
        p->ki = kp * (dt/ti);
    }
    if (dt != 0.0) {
        p->kd = kp * (td/dt);
    }
}

//-----------------------------------------------------------------------------

// Set clamp values for the integral term - prevent integral windup.
void pid_i_limit(PID_CTRL *p, float val)
{
    if (p->ki != 0.0) {
        p->i_limit = val / p->ki;
    }
}

// Set error limit value - give up if error is too large
void pid_e_limit(PID_CTRL *p, float val)
{
    p->e_limit = val;
}

//-----------------------------------------------------------------------------
