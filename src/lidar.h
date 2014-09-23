//-----------------------------------------------------------------------------
/*

Neato XV11 LIDAR Driver for STM32F4 platforms

*/
//-----------------------------------------------------------------------------

#ifndef LIDAR_H
#define LIDAR_H

//-----------------------------------------------------------------------------

#include "pwm.h"
#include "pid.h"
#include "usart.h"

//-----------------------------------------------------------------------------
// target rpm for lidar motor

#define LIDAR_RPM 300.0
#define LIDAR_DEFAULT_PWM 0.80

#define LIDAR_PID_KP 0.0
#define LIDAR_PID_TI 0.0
#define LIDAR_PID_TD 0.0
#define LIDAR_PID_DT 0.0

//-----------------------------------------------------------------------------
// lidar frame layout

#define LIDAR_SOF_DELIMITER 0xfa
#define LIDAR_MIN_INDEX 0xa0
#define LIDAR_MAX_INDEX 0xf9
#define SAMPLES_PER_FRAME 4

struct LIDAR_frame {

    uint8_t index; // 0xa0 - 0xf9 (0-89 offset, 4 samples per frame = 360 measurements)
    uint8_t start; // 0xfa
    uint16_t speed; // little endian, rpm = speed/64
    uint32_t sample[SAMPLES_PER_FRAME];
    uint16_t checksum;

} __attribute__((__packed__));

typedef struct LIDAR_frame LIDAR_frame_t;

//-----------------------------------------------------------------------------

typedef struct lidar_driver {

    // frame data
    uint8_t frame[sizeof(LIDAR_frame_t)];
    int idx;
    // motor control
    PID_CTRL pid;
    float current_rpm;
    float desired_rpm;
    // serial control
    USART_t *sio;
    // pwm control
    PWM_t *pwm;
    // range callback
    void (*range)(int angle, uint32_t data);

} LIDAR_t;

//-----------------------------------------------------------------------------
// macros to extract range data information

#define LIDAR_RANGE_E0(x)   (x)
#define LIDAR_RANGE_E1(x)   (x)
#define LIDAR_RANGE_DIST(x) (x)
#define LIDAR_RANGE_SS(x)   (x)

//         uint8_t byte0 = (frame->sample[i] >> 0) & 0xff;
//         uint8_t byte1 = (frame->sample[i] >> 8) & 0xff;
//         uint8_t byte2 = (frame->sample[i] >> 16) & 0xff;
//         uint8_t byte3 = (frame->sample[i] >> 24) & 0xff;
//
//         int e0 = (byte0 & 0x80) >> 7;  // No return/max range/too low of reflectivity
//         int e1 = (byte0 & 0x40) >> 6;  // Object too close, possible poor reading due to proximity kicks in at < 0.6m
//         uint16_t dist = ((byte0 & 0x3F) << 8) + byte1;
//         uint16_t ss = (byte2 << 8) + byte3;

//-----------------------------------------------------------------------------

LIDAR_t *lidar_init(unsigned int idx, USART_t *sio, PWM_t *pwm);
void lidar_run(LIDAR_t *lidar);

//-----------------------------------------------------------------------------

#endif // LIDAR_H

//-----------------------------------------------------------------------------