//-----------------------------------------------------------------------------
/*

GPIO Control for the STM32F4 Discovery Board

Pin Assignments for STM32F429I Discovery Board

PA0 = push button
PA1 = mems_int1
PA2 = mems_int2
PA3 =
PA4 = video
PA5 =
PA6 =
PA7 = acp
PA8 = acp i2c
PA9 =
PA10 =
PA11 =
PA12 =
PA13 =
PA14 =
PA15 = acp

PB0
PB1
PB2
PB3 =
PB4
PB5
PB6 =
PB7 =
PB8
PB9 =
PB10 =
PB11
PB12 = usb
PB13 = usb
PB14 = usb
PB15 = usb

PC0 =
PC1 = ncs_mems_spi
PC2 = video
PC3 =
PC4 = usb
PC5 = usb
PC6 = video
PC7 =
PC8
PC9 = acp i2c
PC10 =
PC11
PC12 =
PC13
PC14 = pc14_osc32_in
PC15 = pc14_osc32_out

PD0 =
PD1 =
PD2 =
PD3 =
PD4 =
PD5 =
PD6 =
PD7 =
PD8 =
PD9 =
PD10 =
PD11 = video
PD12 = video
PD13 = video
PD14 =
PD15 =

PE0 =
PE1 =
PE2 =
PE3 =
PE4 =
PE5 =
PE6 =
PE7 =
PE8 =
PE9 =
PE10 =
PE11 =
PE12 =
PE13 =
PE14 =
PE15 =

PF0 =
PF1 =
PF2 =
PF3 =
PF4 =
PF5 =
PF6 =
PF7 = spi5_sck, video
PF8 = spi5_miso
PF9 = spi5_mosi, video
PF10 = video
PF11 =
PF12 =
PF13 =
PF14 =
PF15 =

PG0 =
PG1 =
PG2 =
PG3 =
PG4 =
PG5 =
PG6 =
PG7 = video
PG8 =
PG9 =
PG10 =
PG11 =
PG12 =
PG13 = led
PG14 = led
PG15 =

PH0 = ph0_osc_in
PH1 = ph1_osc_out

*/
//-----------------------------------------------------------------------------

#ifndef GPIO_H
#define GPIO_H

//-----------------------------------------------------------------------------

#include "stm32f4xx_hal.h"

//-----------------------------------------------------------------------------
// port numbers

#define PORTA 0
#define PORTB 1
#define PORTC 2
#define PORTD 3
#define PORTE 4
#define PORTF 5
#define PORTG 6
#define PORTH 7
#define PORTI 8

//-----------------------------------------------------------------------------
// gpio macros

#define GPIO_NUM(port, pin) ((port << 4) | (pin))
#define GPIO_PORT(n) (n >> 4)
#define GPIO_PIN(n) (n & 0xf)
#define GPIO_BIT(n) (1 << GPIO_PIN(n))
#define GPIO_BASE(n) ((GPIO_TypeDef  *)(GPIOA_BASE + (GPIO_PORT(n) * 0x400)))

//-----------------------------------------------------------------------------
// gpio assignments

// standard board GPIO
#define LED_GREEN       GPIO_NUM(PORTG, 13)
#define LED_RED         GPIO_NUM(PORTG, 14)
#define PUSH_BUTTON     GPIO_NUM(PORTA, 0) // 0 = open, 1 = pressed

//-----------------------------------------------------------------------------
// generic api functions

static inline void gpio_clr(int n)
{
    GPIO_BASE(n)->BSRRH = GPIO_BIT(n);
}

static inline void gpio_set(int n)
{
    GPIO_BASE(n)->BSRRL = GPIO_BIT(n);
}

static inline void gpio_toggle(int n)
{
    GPIO_BASE(n)->ODR ^= GPIO_BIT(n);
}

static inline int gpio_rd(int n)
{
    return (GPIO_BASE(n)->IDR >> GPIO_PIN(n)) & 1;
}

static inline int gpio_rd_inv(int n)
{
    return (~(GPIO_BASE(n)->IDR) >> GPIO_PIN(n)) & 1;
}

void gpio_init(void);

//-----------------------------------------------------------------------------
// The input gpios are spread out across several ports. We read and pack them into a
// single uint32_t and debounce them together.

// debounced input switches
#define PUSH_BUTTON_BIT 0

static inline uint32_t debounce_input(void)
{
    // pack the gpio inputs to be debounced into the uint32_t debounce state
    return (gpio_rd(PUSH_BUTTON) << PUSH_BUTTON_BIT);
}

//-----------------------------------------------------------------------------

#endif // GPIO_H

//-----------------------------------------------------------------------------
