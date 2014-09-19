//-----------------------------------------------------------------------------
/*

PID - Proportional, Integral, Derivative Controller

*/
//-----------------------------------------------------------------------------

#ifndef PID_H
#define PID_H

//-----------------------------------------------------------------------------

typedef struct pid_control {

    float isum;     // integral sum
    float prev;     // previous error

    float kp;       // proportional gain
    float ki;       // integral gain
    float kd;       // derivative gain

    float i_limit;  // integral limit
    float e_limit;  // error limit

} PID_CTRL;

//-----------------------------------------------------------------------------

void pid(PID_CTRL *p, float error, float *out, int debug);
void pid_i_limit(PID_CTRL *p, float val);
void pid_e_limit(PID_CTRL *p, float val);
void pid_init1(PID_CTRL *p, float kp, float ki, float kd);
void pid_init2(PID_CTRL *p, float kp, float ti, float td, float dt);

//-----------------------------------------------------------------------------

#endif // PID_H

//-----------------------------------------------------------------------------