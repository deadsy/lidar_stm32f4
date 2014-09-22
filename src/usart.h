//-----------------------------------------------------------------------------
/*

USART Driver

*/
//-----------------------------------------------------------------------------

#ifndef USART_H
#define USART_H

//-----------------------------------------------------------------------------

// polled or interrupt based driver
//#define USART_POLLED

//-----------------------------------------------------------------------------

#define USART_TX_BUFFER_SIZE 16 // must be a power of 2
#define USART_RX_BUFFER_SIZE 256 // must be a power of 2

typedef struct usart_driver {

    USART_TypeDef *usart;
    int irq;

#ifndef USART_POLLED
    uint8_t txbuf[USART_TX_BUFFER_SIZE];
    uint8_t rxbuf[USART_RX_BUFFER_SIZE];
    int tx_rd;
    int tx_wr;
    volatile int tx_n;
    int rx_rd;
    int rx_wr;
    volatile int rx_n;
#endif

    uint8_t (*rx)(void);
    int (*test_rx)(void);
    void (*tx)(uint8_t c);

} USART_t;

//-----------------------------------------------------------------------------

USART_t *usart_init(unsigned int idx);

//-----------------------------------------------------------------------------

#endif // USART_H

//-----------------------------------------------------------------------------
