//-----------------------------------------------------------------------------
/*

GPIO Control for the STM32F4 Discovery Board

Pin Assignments for STM32F429I Discovery Board

PA0 = push button
PA1 = mems_int1
PA2 = mems_int2
PA3 = lcd b5
PA4 = video
PA5 =
PA6 = lcd g2
PA7 = acp_rst
PA8 = i2c3_scl
PA9 =
PA10 =
PA11 = lcd r4
PA12 = lcd r5
PA13 =
PA14 =
PA15 = tp_int1

PB0 = lcd r3
PB1 = lcd r6
PB2 =
PB3 =
PB4 =
PB5 = sdcke1
PB6 = sdne1
PB7 =
PB8 = lcd b6
PB9 = lcd b7
PB10 = lcd g4
PB11 = lcd g5
PB12 = usb otg_fs_id
PB13 = usb vbus_fs
PB14 = usb otg_fs_dm
PB15 = usb otg_fs_dp

PC0 = sdnwe
PC1 = ncs_mems_spi
PC2 = video
PC3 =
PC4 = usb otg_fs_pso
PC5 = usb otg_fs_oc
PC6 = video
PC7 = lcd g6
PC8 =
PC9 = i2c3_sda
PC10 = lcd r2
PC11 =
PC12 =
PC13 =
PC14 = pc14_osc32_in
PC15 = pc14_osc32_out

PD0 = sdram d2
PD1 = sdram d3
PD2 = im0
PD3 = lcd g7
PD4 = im1
PD5 = im2
PD6 = lcd b2
PD7 = im3
PD8 = sdram d13
PD9 = sdram d14
PD10 = sdram d15
PD11 = video
PD12 = video
PD13 = video
PD14 = sdram d0
PD15 = sdram d1

PE0 = nbl0
PE1 = nbl1
PE2 =
PE3 =
PE4 =
PE5 =
PE6 =
PE7 = sdram d4
PE8 = sdram d5
PE9 = sdram d6
PE10 = sdram d7
PE11 = sdram d8
PE12 = sdram d9
PE13 = sdram d10
PE14 = sdram d11
PE15 = sdram d12

PF0 = sdram a0
PF1 = sdram a1
PF2 = sdram a2
PF3 = sdram a3
PF4 = sdram a4
PF5 = sdram a5
PF6 =
PF7 = spi5_sck, video
PF8 = spi5_miso
PF9 = spi5_mosi, video
PF10 = video
PF11 = sdnras
PF12 = sdram a6
PF13 = sdram a7
PF14 = sdram a8
PF15 = sdram a9

PG0 = sdram a10
PG1 = sdram a11
PG2 =
PG3 =
PG4 = ba0
PG5 = ba1
PG6 = lcd r7
PG7 = video
PG8 = sdclk
PG9 =
PG10 = lcd g3
PG11 = lcd b3
PG12 = lcd b4
PG13 = green led
PG14 = red led
PG15 = sdncas

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

// serial port
#define UART_TX         GPIO_NUM(PORTA, 9)
#define UART_RX         GPIO_NUM(PORTA, 10)

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
