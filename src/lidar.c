//-----------------------------------------------------------------------------
/*

Neato XV11 LIDAR Driver for STM32F4 platforms

*/
//-----------------------------------------------------------------------------

#include <string.h>
#include <stdint.h>

#include "lidar.h"

//-----------------------------------------------------------------------------

#define NUM_LIDARS 1
static LIDAR_t lidars[NUM_LIDARS];

//-----------------------------------------------------------------------------

static uint16_t flip16(uint16_t data) {
    return (data >> 8) | (data << 8);
}

//-----------------------------------------------------------------------------
// return the checksum for a LIDAR data frame

static uint16_t calc_checksum(uint16_t *data) {

    uint32_t cs = 0;
    int i;

    for (i = 0; i < 10; i ++) {
        cs = (cs << 1) + flip16(data[i]);
    }

    cs = ((cs & 0x7fff) + (cs >> 15)) & 0x7fff;
    return (uint16_t)cs;
}

//-----------------------------------------------------------------------------
// process a frame of lidar data

static void lidar_frame(LIDAR_t *lidar) {

    LIDAR_frame_t *frame = (LIDAR_frame_t *)lidar->frame;
    int i;

    if (lidar->range == 0) {
        // no callback to take the data
        return;
    }

    if (flip16(frame->checksum) != calc_checksum((uint16_t *)lidar->frame)) {
        // checksum failed
        return;
    }

    lidar->current_rpm = flip16(frame->speed)/ 64.0;

    for (i = 0; i < SAMPLES_PER_FRAME; i ++) {
        int angle = ((frame->index - LIDAR_MIN_INDEX) * 4) + i;
        lidar->range(angle, frame->sample[i]);
    }
}

//-----------------------------------------------------------------------------
// read a frame from the serial port

void lidar_rx(LIDAR_t *lidar) {

    while (1) {
        if (usart_test_rx(lidar->sio) == 0) {
            // no more rx data
            return;
        }

        lidar->frame[lidar->idx] = usart_rx(lidar->sio);
        lidar->idx ++;

        if (lidar->idx == 2) {
            if ((lidar->frame[1] != LIDAR_SOF_DELIMITER) ||
                (lidar->frame[0] < LIDAR_MIN_INDEX) ||
                (lidar->frame[0] > LIDAR_MAX_INDEX)) {
                // not synched to the start of frame
                lidar->idx = 0;
            }
        } else if (lidar->idx == sizeof(LIDAR_frame_t)) {
            lidar_frame(lidar);
            lidar->idx = 0;
        }
    }
}

//-----------------------------------------------------------------------------
// motor control

void lidar_motor_on(LIDAR_t *lidar) {
    lidar->desired_rpm = LIDAR_RPM;
    lidar->current_rpm = -1.0;
    pwm_set(lidar->pwm, LIDAR_DEFAULT_PWM);
}

void lidar_motor_off(LIDAR_t *lidar) {
    lidar->desired_rpm = 0.0;
    lidar->current_rpm = -1.0;
    pwm_set(lidar->pwm, 0.0);
}

// must be called periodically at the interval used for PID constant calculations
void lidar_motor_ctrl(LIDAR_t *lidar) {

    if ((lidar->desired_rpm = 0.0) || (lidar->current_rpm < 0.0)) {
        // Don't run PID control unless we have a non-zero
        // desired rpm and a good value for the current rpm
        return;
    }

    // run the PID control
    float out;
    pid(&lidar->pid, lidar->desired_rpm - lidar->current_rpm, &out, 0);
    pwm_delta(lidar->pwm, out);
}

//-----------------------------------------------------------------------------

void lidar_run(LIDAR_t *lidar) {
    // TODO
}

//-----------------------------------------------------------------------------
// Initialise the LIDAR driver

LIDAR_t *lidar_init(unsigned int idx, USART_t *sio, PWM_t *pwm) {

    if (idx >= NUM_LIDARS) {
        return 0;
    }

    LIDAR_t *lidar = &lidars[idx];
    memset(lidar, 0, sizeof(LIDAR_t));

    lidar->sio = sio;
    lidar->pwm = pwm;

    pid_init2(&lidar->pid, LIDAR_PID_KP, LIDAR_PID_TI, LIDAR_PID_TD, LIDAR_PID_DT);
    lidar_motor_off(lidar);
    return lidar;
}

//-----------------------------------------------------------------------------
