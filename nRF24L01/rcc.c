#include "rcc.h"

void rcc_enable_gpio(uint32_t gpios)
{
  uint32_t *rcc_enable_offset = (uint32_t *)(RCC_BASE + RCC_AHB1ENR);
  
  *rcc_enable_offset |= gpios;
}

void rcc_enable_spi(uint32_t spis)
{
  uint32_t *rcc_enable_offset = (uint32_t *)(RCC_BASE + RCC_APB2ENR);
  
  *rcc_enable_offset |= spis;
}

void rcc_enable_uart(void)
{
  uint32_t *rcc_enable_offset = (uint32_t *)(RCC_BASE + RCC_APB2ENR);
  
  *rcc_enable_offset |= RCC_USART_1_EN;
}

