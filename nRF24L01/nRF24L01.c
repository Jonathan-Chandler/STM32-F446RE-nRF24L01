#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include "rcc.h"
#include "gpio.h"
#include "spi.h"
#include "radio.h"

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

#define USART_CR1       0x0C
#define USART_CR1_UE    (1 << 13)     // USART enable
#define USART_CR1_TE    (1 << 3)      // transmitter enable
#define USART_CR1_RE    (1 << 2)      // receiver enable

#define USART_CR2       0x10
#define USART_CR3       0x14
#define USART_GTPR      0x18


uint32_t *usart_cr1 = (uint32_t *)(USART_1_BASE + USART_CR1);
*usart_cr1 |= USART_CR1_UE;

// 1. Enable the USART by writing the UE bit in USART_CR1 register to 1.
// 2. Program the M bit in USART_CR1 to define the word length.
// 3. Program the number of stop bits in USART_CR2.
// 4. Select DMA enable (DMAR) in USART_CR3 if multibuffer communication is to take place. Configure the DMA register as explained in multibuffer communication. STEP 3
// 5. Select the desired baud rate using the baud rate register USART_BRR
// 6. Set the RE bit USART_CR1. This enables the receiver that begins searching for a start bit.

// The RXNE bit is set. It indicates that the content of the shift register is transferred to the RDR. In other words, data has been received and can be read (as well as its associated error flags).
// An interrupt is generated if the RXNEIE bit is set.
// The error flags can be set if a frame error, noise or an overrun error has been detected during reception.
// In multibuffer, RXNE is set after every byte received and is cleared by the DMA read to the Data Register.
// In single buffer mode, clearing the RXNE bit is performed by a software read to the USART_DR register. The RXNE flag can also be cleared by writing a zero to it. The RXNE bit must be cleared before the end of the reception of the next character to avoid an overrun error.

void init(void);
void configure_usart(void);

int main(void)
{
  init();
  uint8_t data[50] = {0};

  enable_gpio(GPIO_A, GPIO_0);      // slave select
  enable_gpio(GPIO_A, GPIO_1);      // chip enable

  radio_configure();

  while (1) 
  {
    uint32_t i = 0;
    memset(data, 0, sizeof(data));

    for (i = 0; i < 5000000; i++) 
    {
      __asm__("nop");
    }

    if (radio_rx_waiting() != 0x7)
    {
      radio_recv(data);
    }
  }

  return 0;
}

void init(void)
{
  uint32_t enabled_gpio = RCC_GPIO_A | RCC_GPIO_B | RCC_GPIO_C;
  uint32_t enabled_spi = RCC_SPI_1_EN;
  rcc_enable_gpio(enabled_gpio);
  rcc_enable_spi(enabled_spi);

  configure_gpio(GPIO_A, GPIO_0, GPIO_MODE_OUT, GPIO_PUPD_PULL);      // slave select
  configure_gpio(GPIO_A, GPIO_1, GPIO_MODE_OUT, GPIO_PUPD_PULL);      // chip enable

  configure_spi();
}

void configure_usart(void)
{
  uint32_t *usart_cr1_register = (uint32_t *)(USART_1_BASE + USART_CR1);
  uint32_t *usart_cr2_register = (uint32_t *)(USART_1_BASE + USART_CR2);
  uint32_t *usart_cr3_register = (uint32_t *)(USART_1_BASE + USART_CR3);
  uint32_t *gpio_afh_a_register = (uint32_t *)(GPIO_A + GPIO_AFRH);

  configure_gpio(GPIO_A, GPIO_9, GPIO_MODE_ALT, GPIO_PUPD_PULL);    // D12 = MISO
  *gpio_afh_a_register |= (0x7 << 1*4);                                   // A6 = AF6 = SPI1_MISO
  configure_gpio(GPIO_A, GPIO_10, GPIO_MODE_ALT, GPIO_PUPD_PULL);    // D13 = SCK
  *gpio_afh_a_register |= (0x7 << 2*4);                                   // A5 = AF5 = SPI1_SCJ
  configure_gpio(GPIO_A, GPIO_11, GPIO_MODE_ALT, GPIO_PUPD_PULL);    // A2 = PA4 = CS
  *gpio_afh_a_register |= (0x7 << 3*4);                                   // A5 = AF5 = SPI1_SCJ
  configure_gpio(GPIO_A, GPIO_12, GPIO_MODE_ALT, GPIO_PUPD_PULL);    // D11 = MOSI
  *gpio_afh_a_register |= (0x7 << 5*4);                                   // A7 = AF7 = SPI1_MOSI
}

