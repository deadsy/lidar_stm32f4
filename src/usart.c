//-----------------------------------------------------------------------------
/*

USART Driver

*/
//-----------------------------------------------------------------------------

#include <string.h>

#include "stm32f4xx_hal.h"
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
    val &= ~(USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE | USART_SR_IDLE |\
        USART_SR_RXNE | USART_SR_TC | USART_SR_TXE | USART_SR_LBD | USART_SR_CTS);
    usart->SR= val;

    set_baud_rate(usart, 115200);

    // GTPR - no changes
    val = usart->GTPR;
    usart->GTPR = val;

    // enable the uart
    usart->CR1 |= USART_CR1_UE;
}

//-----------------------------------------------------------------------------

#ifdef USART_POLLED

// Polled driver

static int usart_test_rx(USART_t *ptr) {
    return (ptr->usart->SR & USART_SR_RXNE) != 0;
}

static uint8_t usart_rx(USART_t *ptr) {
    return ptr->usart->DR & 0xff;
}

static void usart_tx(USART_t *ptr, uint8_t c) {
    USART_TypeDef* const usart = ptr->usart;
    while ((usart->SR & USART_SR_TXE) == 0);
    usart->DR = c;
}

#else

// ISR based driver with circular tx/rx FIFOs

static int usart_test_rx(USART_t *ptr) {
    return ptr->rx_n != 0;
}

static uint8_t usart_rx(USART_t *ptr) {
    NVIC_DisableIRQ(ptr->irq);
    uint8_t c = ptr->rxbuf[ptr->rx_rd];
    ptr->rx_rd = (ptr->rx_rd + 1) & (USART_RX_BUFFER_SIZE - 1);
    ptr->rx_n -= 1;
    NVIC_EnableIRQ(ptr->irq);
    return c;
}

static void usart_tx(USART_t *ptr, uint8_t c) {
    // wait for space
    while (ptr->tx_n == (USART_TX_BUFFER_SIZE - 1));
    NVIC_DisableIRQ(ptr->irq);
    if(ptr->tx_n == 0) {
    }
    // Put the character into the Tx buffer.
    ptr->txbuf[ptr->tx_wr] = c;
    ptr->tx_wr = (ptr->tx_wr + 1) & (USART_TX_BUFFER_SIZE - 1);
    ptr->tx_n += 1;
    NVIC_EnableIRQ(ptr->irq);
}

static void usart_isr(USART_t *ptr) {
}

#endif

//-----------------------------------------------------------------------------
// 0/USART1

static uint8_t rx_0(void) {
    return usart_rx(&usarts[0]);
}

static int test_rx_0(void) {
    return usart_test_rx(&usarts[0]);
}

static void tx_0(uint8_t c) {
    return usart_tx(&usarts[0], c);
}

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
        ptr->rx = rx_0;
        ptr->test_rx = test_rx_0;
        ptr->tx = tx_0;
    } else {
        return 0;
    }

    usart_hw_init(ptr);
    return ptr;
}

//-----------------------------------------------------------------------------
