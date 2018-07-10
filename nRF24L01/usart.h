#ifndef INCLUDE_USART_H
#define INCLUDE_USART_H

#include "gpio.h"
#include <stdbool.h>

#define USART_1_BASE  0x40011000
#define USART_2_BASE  0x40004400
#define USART_3_BASE  0x40004800
#define USART_6_BASE  0x40011400
#define UART_4_BASE   0x40004C00
#define UART_5_BASE   0x40005000

#define USART_SR        0x00
#define USART_SR_TXE    (1 << 7)      // transmit register empty; An interrupt is generated if the TXEIE bit =1 in the USART_CR1 register. It is cleared by a write to the USART_DR register.
#define USART_SR_TC     (1 << 6)      // transmission complete
#define USART_SR_TXNE   (1 << 5)      // read data register not empty (1: rx data ready)
#define USART_SR_IDLE   (1 << 4)      // idle line detected

#define USART_DR        0x04
#define USART_BRR       0x08
#define USART_BRR_FRACTION_MASK 0xF
#define USART_BRR_FRACTION_OFFS 0
#define USART_BRR_MANTISSA_MASK (0xFFF << 4)
#define USART_BRR_MANTISSA_OFFS 4

#define USART_CR1       0x0C
#define USART_CR1_UE    (1 << 13)     // USART enable
#define USART_CR1_TE    (1 << 3)      // transmitter enable
#define USART_CR1_RE    (1 << 2)      // receiver enable

#define USART_CR2       0x10
#define USART_CR3       0x14
#define USART_GTPR      0x18

void configure_usart(void);

bool uart_tx_empty(void);
void uart_tx(char data);

#endif
