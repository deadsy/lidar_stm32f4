//-----------------------------------------------------------------------------
/*

USART Driver

*/
//-----------------------------------------------------------------------------

#ifndef USART_H
#define USART_H

//-----------------------------------------------------------------------------

#define USART_TX_BUFFER_SIZE 16
#define USART_RX_BUFFER_SIZE 256

typedef struct usart_driver {

    USART_TypeDef *usart;

    uint8_t txbuf[USART_TX_BUFFER_SIZE];
    uint8_t rxbuf[USART_RX_BUFFER_SIZE];
    int tx_rd;
    int tx_wr;
    int rx_rd;
    int rx_wr;

    uint8_t (*rx)(void);
    int (*test_rx)(void);
    void (*tx)(uint8_t c);

} USART_t;

//-----------------------------------------------------------------------------

USART_t *usart_init(unsigned int idx);

//-----------------------------------------------------------------------------

#endif // USART_H

//-----------------------------------------------------------------------------
