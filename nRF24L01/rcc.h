#ifndef INCLUDE_RCC_H
#define INCLUDE_RCC_H
#include <stdint.h>

#define RCC_BASE      0x40023800
#define RCC_AHB1ENR   0x30            // 0=GPIOA ... 7=GPIOH enable
#define RCC_GPIO_A    (1 << 0)
#define RCC_GPIO_B    (1 << 1)
#define RCC_GPIO_C    (1 << 2)
#define RCC_GPIO_D    (1 << 3)
#define RCC_GPIO_E    (1 << 4)
#define RCC_GPIO_F    (1 << 5)
#define RCC_GPIO_G    (1 << 6)
#define RCC_GPIO_H    (1 << 7)

#define RCC_APB1ENR   0x40            
#define RCC_SPI_2_EN  (1 << 14)
#define RCC_SPI_3_EN  (1 << 15)

#define RCC_APB2ENR   0x44            
#define RCC_SPI_1_EN  (1 << 12)
#define RCC_SPI_4_EN  (1 << 13)

void rcc_enable_gpio(uint32_t gpios);
void rcc_enable_spi(uint32_t spis);

#endif
