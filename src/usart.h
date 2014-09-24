//-----------------------------------------------------------------------------
/*

USART Driver

*/
//-----------------------------------------------------------------------------

#ifndef USART_H
#define USART_H

//-----------------------------------------------------------------------------

#include "stm32f4xx_hal.h"

//-----------------------------------------------------------------------------

// polled or interrupt based driver
//#define USART_POLLED

//-----------------------------------------------------------------------------

#define USART_TX_BUFFER_SIZE 4 // must be a power of 2
#define USART_RX_BUFFER_SIZE 16 // must be a power of 2

typedef struct usart_driver {

    USART_TypeDef *usart;
    int irq;

#ifndef USART_POLLED
    uint8_t txbuf[USART_TX_BUFFER_SIZE];
    uint8_t rxbuf[USART_RX_BUFFER_SIZE];
    volatile int tx_rd;
    volatile int tx_wr;
    volatile int rx_rd;
    volatile int rx_wr;

    int total_ints;
    int rx_overflow;

#endif

} USART_t;

//-----------------------------------------------------------------------------

USART_t *usart_init(unsigned int idx);
int usart_test_rx(USART_t *ptr);
uint8_t usart_rx(USART_t *ptr);
void usart_tx(USART_t *ptr, uint8_t c);
void usart_puts(USART_t *ptr, const char *s);

//-----------------------------------------------------------------------------

#endif // USART_H

//-----------------------------------------------------------------------------
