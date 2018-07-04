#include "gpio.h"

void configure_gpio(uint32_t gpio_group, uint32_t gpio_id, uint32_t gpio_mode, uint32_t gpio_pupd_mode)
{
  uint32_t *gpio_mode_select = (uint32_t *)(gpio_group + GPIO_MODER);
  uint32_t gpio_mode_setting = gpio_mode << (gpio_id * 2);
  
  uint32_t *gpio_pupd_select = (uint32_t *)(gpio_group + GPIO_PUPDR);
  uint32_t gpio_pupd_setting = gpio_pupd_mode << (gpio_id * 2);

  *gpio_mode_select |= gpio_mode_setting;
  *gpio_pupd_select |= gpio_pupd_setting;
}

void enable_gpio(uint32_t gpio_group, uint32_t gpio_id)
{
  uint32_t *gpio_out_data_select = (uint32_t *)(gpio_group + GPIO_OUT_DR);
  uint32_t gpio_out_data_setting = 1 << gpio_id;

  *gpio_out_data_select |= gpio_out_data_setting;
}

void disable_gpio(uint32_t gpio_group, uint32_t gpio_id)
{
  uint32_t *gpio_out_data_select = (uint32_t *)(gpio_group + GPIO_OUT_DR);
  uint32_t gpio_out_data_setting = 1 << gpio_id;

  *gpio_out_data_select &= ~gpio_out_data_setting;
}

void toggle_gpio(uint32_t gpio_group, uint32_t gpio_id)
{
  uint32_t *gpio_out_data_select = (uint32_t *)(gpio_group + GPIO_OUT_DR);
  uint32_t gpio_out_data_setting = 1 << gpio_id;

  if (*gpio_out_data_select & gpio_out_data_setting)
    disable_gpio(gpio_group, gpio_id);
  else
    enable_gpio(gpio_group, gpio_id);
}

