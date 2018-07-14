#include "usart.h"

void configure_usart(void)
{
  volatile uint32_t *usart_cr1_register = (uint32_t *)(USART_1_BASE + USART_CR1);
  volatile uint32_t *usart_brr_register = (uint32_t *)(USART_1_BASE + USART_BRR);
  volatile uint32_t *gpio_afh_a_register = (uint32_t *)(GPIO_A + GPIO_AFRH);

  configure_gpio(GPIO_A, GPIO_9, GPIO_MODE_ALT, GPIO_PUPD_PULL);      // 
  *gpio_afh_a_register |= (0x7 << 1*4);                               // PA9 = UART1_TX
  configure_gpio(GPIO_A, GPIO_10, GPIO_MODE_ALT, GPIO_PUPD_PULL);     // 
  *gpio_afh_a_register |= (0x7 << 2*4);                               // PA10 = UART1_RX

  //   usart only
  //   configure_gpio(GPIO_A, GPIO_11, GPIO_MODE_ALT, GPIO_PUPD_PULL);    // 
  //   *gpio_afh_a_register |= (0x7 << 3*4);                              // PA11 = UART1_CTS
  //   configure_gpio(GPIO_A, GPIO_12, GPIO_MODE_ALT, GPIO_PUPD_PULL);    // 
  //   *gpio_afh_a_register |= (0x7 << 5*4);                              // PA12 = UART1_RTS

  // USART enable | RX enable | TX enable
  *usart_cr1_register |= (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);

  // bitrate register = (2*clock + baud) / 2*baud
  *usart_brr_register = ((16000000 * 2) + 9600) / (2*9600);
}

void uart_tx_str(char *data)
{
  while (*data)
  {
    uart_tx_byte(*data);
    data++;
  }

  // CR + LF
  uart_tx_byte(10);
  uart_tx_byte(13);
}

void uart_tx_byte(char data)
{
  volatile uint32_t *usart_data_register = (uint32_t *)(USART_1_BASE + USART_DR);

  // wait until tx buffer empty
  while (!uart_tx_empty())
  {
    __asm__("nop");
  }

  *usart_data_register = data;
}

bool uart_tx_empty(void)
{
  volatile uint32_t *usart_status_register = (uint32_t *)(USART_1_BASE + USART_SR);

  if (*usart_status_register & USART_SR_TXE)
    return true;

  return false;
}
