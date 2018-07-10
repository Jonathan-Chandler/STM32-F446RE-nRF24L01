#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include "rcc.h"
#include "gpio.h"
#include "spi.h"
#include "radio.h"
#include "usart.h"

void init(void);

int main(void)
{
  init();
  uint8_t data[50] = {0};
  char character = 'U'; 

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

    if (uart_tx_empty())
    {
      uart_tx(character);
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
  rcc_enable_uart();

  configure_gpio(GPIO_A, GPIO_0, GPIO_MODE_OUT, GPIO_PUPD_PULL);      // slave select
  configure_gpio(GPIO_A, GPIO_1, GPIO_MODE_OUT, GPIO_PUPD_PULL);      // chip enable

  configure_spi();
  configure_usart();
}

