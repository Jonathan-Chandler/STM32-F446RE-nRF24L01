#ifndef INCLUDE_GPIO_H
#define INCLUDE_GPIO_H
#include <stdint.h>

#define GPIO_A              0x40020000
#define GPIO_B              0x40020400
#define GPIO_C              0x40020800
#define GPIO_D              0x40020C00
#define GPIO_E              0x40021000
#define GPIO_F              0x40021400
#define GPIO_G              0x40021800
#define GPIO_H              0x40021C00

#define GPIO_MODER          0x00          // 00 - input (reset state); 01 - general purpose output mode; 10 - alternate function mode; 11 - analogue mode
#define GPIO_MODE_IN        0x0           // 00 - input (reset state); 01 - general purpose output mode; 10 - alternate function mode; 11 - analogue mode
#define GPIO_MODE_OUT       0x1           // 00 - input (reset state); 01 - general purpose output mode; 10 - alternate function mode; 11 - analogue mode
#define GPIO_MODE_ALT       0x2           // 00 - input (reset state); 01 - general purpose output mode; 10 - alternate function mode; 11 - analogue mode
#define GPIO_MODE_ANALOGUE  0x3           // 00 - input (reset state); 01 - general purpose output mode; 10 - alternate function mode; 11 - analogue mode

#define GPIO_OTYPER         0x04          // 0 - output push-pull; 1 - open drain; bits 31:16 reserved, must not be modified
#define GPIO_OSPEED         0x08          // 00 - low speed; 01 medium; 10 fast; 11 high
#define GPIO_PUPDR          0x0C          // 00 - no pull/push; 01 - push up; 10 - pull down; 11 - reserved
#define GPIO_PUPD_NONE      0x0           
#define GPIO_PUPD_PUSH      0x1           
#define GPIO_PUPD_PULL      0x2           
#define GPIO_PUPD_RESERVED  0x3           

#define GPIO_IN_DR          0x10          // 00..15 = input data bits; 16..31 = reserved
#define GPIO_OUT_DR         0x14          // 00..15 = data bits; 16..31 = reserved
#define GPIO_SET_OFFSET     0x18          // 00..15 = set bits; 16..31 = reset bits
#define GPIO_LOCK_REG       0x1C          // 00..15 = set bits; 16..31 = reset bits
#define GPIO_AFRL           0x20          // GPIO 0-7 alt function low register; 4 bits/pin; AFx = 0x0 - 0xF
#define GPIO_AFRH           0x24          // GPIO 8-15 alt function high register; 4 bits/pin; AFx = 0x0 - 0xF

#define GPIO_0        0x0
#define GPIO_1        0x1
#define GPIO_2        0x2
#define GPIO_3        0x3
#define GPIO_4        0x4
#define GPIO_5        0x5
#define GPIO_6        0x6
#define GPIO_7        0x7
#define GPIO_8        0x8
#define GPIO_9        0x9
#define GPIO_10       0xA
#define GPIO_11       0xB
#define GPIO_12       0xC
#define GPIO_13       0xD
#define GPIO_14       0xE
#define GPIO_15       0xF

void configure_gpio(uint32_t gpio_group, uint32_t gpio_pin, uint32_t gpio_mode, uint32_t gpio_pupd_mode);
void enable_gpio(uint32_t gpio_group, uint32_t gpio_id);
void disable_gpio(uint32_t gpio_group, uint32_t gpio_id);
void toggle_gpio(uint32_t gpio_group, uint32_t gpio_id);

#endif
