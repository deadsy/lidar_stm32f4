//-----------------------------------------------------------------------------
/*

USART Driver

*/
//-----------------------------------------------------------------------------

#include <string.h>

#include "usart.h"

//-----------------------------------------------------------------------------

#define NUM_USARTS 1
static USART_t usarts[NUM_USARTS];

//-----------------------------------------------------------------------------

// enable the peripheral clock for the usart
static void enable_usart_clock(USART_TypeDef *usart)
{
    if (usart == USART1) {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    } else if (usart == USART2) {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    } else if (usart == USART3) {
        RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    } else if (usart == UART4) {
        RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
    } else if (usart == UART5) {
        RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
    } else if (usart == USART6) {
        RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
    }
}

//-----------------------------------------------------------------------------

static void set_baud_rate(USART_TypeDef *usart, int baud)
{
    if (usart->CR1 & USART_CR1_OVER8) {
        if (usart == USART1 || usart == USART6) {
            usart->BRR = __UART_BRR_SAMPLING8(HAL_RCC_GetPCLK2Freq(), baud);
        } else {
            usart->BRR = __UART_BRR_SAMPLING8(HAL_RCC_GetPCLK1Freq(), baud);
        }
    } else {
        if (usart == USART1 || usart == USART6) {
            usart->BRR = __UART_BRR_SAMPLING16(HAL_RCC_GetPCLK2Freq(), baud);
        } else {
            usart->BRR = __UART_BRR_SAMPLING16(HAL_RCC_GetPCLK1Freq(), baud);
        }
    }
}

//-----------------------------------------------------------------------------

static void usart_hw_init(USART_t *ptr)
{
    USART_TypeDef* usart = ptr->usart;
    uint32_t val;

    // enable peripheral clock
    enable_usart_clock(usart);

    // disable the uart
    usart->CR1 &= ~USART_CR1_UE;

    // set CR1
    val = usart->CR1;
    val &= ~(USART_CR1_SBK | USART_CR1_RWU | USART_CR1_IDLEIE | USART_CR1_RXNEIE |\
        USART_CR1_TCIE | USART_CR1_TXEIE | USART_CR1_PEIE | USART_CR1_PS | USART_CR1_PCE |\
        USART_CR1_WAKE | USART_CR1_M | USART_CR1_UE | USART_CR1_OVER8);
    val |= (USART_CR1_RE | USART_CR1_TE);
    usart->CR1= val;

    // set CR2
    val = usart->CR2;
    val &= ~(USART_CR2_ADD | USART_CR2_LBDL | USART_CR2_LBDIE | USART_CR2_LBCL |\
        USART_CR2_CPHA | USART_CR2_CPOL | USART_CR2_CLKEN | USART_CR2_STOP |\
        USART_CR2_LINEN);
    usart->CR2= val;

    // set CR3
    val = usart->CR3;
    val &= ~(USART_CR3_EIE | USART_CR3_IREN | USART_CR3_IRLP | USART_CR3_HDSEL |\
        USART_CR3_NACK | USART_CR3_SCEN | USART_CR3_DMAR | USART_CR3_DMAT |\
        USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_CTSIE | USART_CR3_ONEBIT);
    usart->CR3= val;

    // clear the status register
    val = usart->SR;
    val &= ~(USART_SR_RXNE | USART_SR_TC | USART_SR_LBD | USART_SR_CTS);
    usart->SR= val;

    set_baud_rate(usart, 115200);

    // GTPR - no changes
    val = usart->GTPR;
    usart->GTPR = val;

#ifndef USART_POLLED
    // turn on interrupts
    usart->CR1 |= USART_CR1_RXNEIE;
    HAL_NVIC_SetPriority(ptr->irq, 1, 0);
    NVIC_EnableIRQ(ptr->irq);
#endif

    // enable the uart
    usart->CR1 |= USART_CR1_UE;
}

//-----------------------------------------------------------------------------
#ifdef USART_POLLED
// Polled driver

int usart_test_rx(USART_t *ptr) {
    return (ptr->usart->SR & USART_SR_RXNE) != 0;
}

uint8_t usart_rx(USART_t *ptr) {
    return ptr->usart->DR;
}

void usart_tx(USART_t *ptr, uint8_t c) {
    USART_TypeDef* const usart = ptr->usart;
    while ((usart->SR & USART_SR_TXE) == 0);
    usart->DR = c;
}

//-----------------------------------------------------------------------------
#else
// ISR based driver with circular tx/rx FIFOs

#define inc_mod(x, size) (x) = ((x) + 1) & ((size) - 1)

#define UNUSED __attribute__((unused))

int usart_test_rx(USART_t *ptr) {
    return ptr->rx_n != 0;
}

uint8_t usart_rx(USART_t *ptr) {
    NVIC_DisableIRQ(ptr->irq);
    uint8_t c = ptr->rxbuf[ptr->rx_rd];
    inc_mod(ptr->rx_rd, USART_RX_BUFFER_SIZE);
    ptr->rx_n -= 1;
    NVIC_EnableIRQ(ptr->irq);
    return c;
}

void usart_tx(USART_t *ptr, uint8_t c) {
    // wait for space
    while (ptr->tx_n == (USART_TX_BUFFER_SIZE - 1));
    NVIC_DisableIRQ(ptr->irq);
    if(ptr->tx_n == 0) {
        // turn on the Tx empty interrupt
        ptr->usart->CR1 |= USART_CR1_TXEIE;
    }
    // Put the character into the Tx buffer.
    ptr->txbuf[ptr->tx_wr] = c;
    inc_mod(ptr->tx_wr, USART_TX_BUFFER_SIZE);
    ptr->tx_n += 1;
    NVIC_EnableIRQ(ptr->irq);
}

static void usart_isr(USART_t *ptr) {

    USART_TypeDef* usart = ptr->usart;
    uint32_t status = usart->SR;
    uint32_t clr UNUSED;

    // receive
    if (status & USART_SR_RXNE) {
        ptr->rxbuf[ptr->rx_wr] = usart->DR;
        inc_mod(ptr->rx_wr, USART_RX_BUFFER_SIZE);
        ptr->rx_n += 1;
    }

    // transmit
    if (status & USART_SR_TXE) {
        usart->DR = ptr->txbuf[ptr->tx_rd];
        inc_mod(ptr->tx_rd, USART_TX_BUFFER_SIZE);
        ptr->tx_n -= 1;
        if(ptr->tx_n == 0) {
            // turn off the Tx empty interrupt
           usart->CR1 &= ~USART_CR1_TXEIE;
        }
    }

    // parity errors
    if (status & USART_SR_PE) {
        // wait for RXNE = 1
        while ((usart->SR & USART_SR_RXNE) == 0);
        clr = usart->DR;
    }

    // other errors - clear by reading DR
    if (status & (USART_SR_FE | USART_SR_NE | USART_SR_ORE | USART_SR_IDLE)) {
        clr = usart->DR;
    }

    // LIN Break Detection Flag
    if (status & USART_SR_LBD) {
        usart->SR &= ~USART_SR_LBD;
    }

    // CTS Flag
    if (status & USART_SR_CTS) {
        usart->SR &= ~USART_SR_CTS;
    }
}

#endif

//-----------------------------------------------------------------------------
// IQR Handlers

#ifndef USART_POLLED
void USART1_IRQHandler(void) {
    usart_isr(&usarts[0]);
}
#endif

//-----------------------------------------------------------------------------

USART_t *usart_init(unsigned int idx) {

    if (idx >= NUM_USARTS) {
        return 0;
    }

    USART_t *ptr = &usarts[idx];
    memset(ptr, 0, sizeof(USART_t));

    if (idx == 0) {
        ptr->usart = USART1;
        ptr->irq = USART1_IRQn;
    } else {
        return 0;
    }

    usart_hw_init(ptr);
    return ptr;
}

//-----------------------------------------------------------------------------
