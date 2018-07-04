#include "spi.h"

void configure_spi(void)
{
  uint32_t *spi_cr1_register = (uint32_t *)(SPI_1_BASE + SPI_CR1);
  uint32_t *spi_cr2_register = (uint32_t *)(SPI_1_BASE + SPI_CR2);
  uint32_t *gpio_afl_a_register = (uint32_t *)(GPIO_A + GPIO_AFRL);

  // CN5 (d10=3; d11=4; d12=5; d13=6)
  configure_gpio(GPIO_A, GPIO_7, GPIO_MODE_ALT, GPIO_PUPD_PULL);    // D11 = MOSI
  *gpio_afl_a_register |= (0x5 << 7*4);                                   // A7 = AF7 = SPI1_MOSI
  configure_gpio(GPIO_A, GPIO_6, GPIO_MODE_ALT, GPIO_PUPD_PULL);    // D12 = MISO
  *gpio_afl_a_register |= (0x5 << 6*4);                                   // A6 = AF6 = SPI1_MISO
  configure_gpio(GPIO_A, GPIO_5, GPIO_MODE_ALT, GPIO_PUPD_PULL);    // D13 = SCK
  *gpio_afl_a_register |= (0x5 << 5*4);                                   // A5 = AF5 = SPI1_SCJ
  configure_gpio(GPIO_A, GPIO_4, GPIO_MODE_ALT, GPIO_PUPD_PULL);    // A2 = PA4 = CS
  *gpio_afl_a_register |= (0x5 << 4*4);                                   // A5 = AF5 = SPI1_SCJ

  *spi_cr1_register |= (SPI_MASTER | (0x4 << 3) | SPI_CR1_SSM | SPI_CR1_SSI);

  *spi_cr2_register |= (SPI_CR2_SSOE);

  *spi_cr1_register |= SPI_CR1_SPE;
}

void spi_read_write(bool read, volatile uint8_t *data, size_t size)
{
  uint32_t *spi_register = (uint32_t *)SPI_1_BASE;
  if (size == 0)
    return;

  disable_gpio(GPIO_A, GPIO_0);     // CSN

  // send command byte
  spi_transfer(spi_register, data[0]);

  // send data bytes (LSB first)
  for (size_t currentByte = 1; currentByte < size; currentByte++)
  {
    if (read)
    {
      // write MOSI then return MISO data
      data[currentByte] = spi_transfer(spi_register, data[currentByte]);
    }
    else
    {
      // write MOSI without reading MISO
      spi_transfer(spi_register, data[currentByte]);
    }
  }

  enable_gpio(GPIO_A, GPIO_0);     // CSN
}

uint8_t spi_transfer(uint32_t *spi_register, uint8_t data)
{
  volatile uint32_t *spi_status_register = (uint32_t *)(SPI_1_BASE + SPI_SR);
  volatile uint32_t *spi_data_register = (uint32_t *)(SPI_1_BASE + SPI_DR);
  if (*spi_register)
  {
  }
  uint8_t recv_data = 0;

  // wait until previous transfer finished
  while (!(*spi_status_register & SPI_SR_TXE));

  // if rx not empty, read until rx buffer is empty
  while ((*spi_status_register & SPI_SR_RXNE))
  {
    // discard
    recv_data = *spi_data_register;
  }

  // send data
  *spi_data_register = data;

  // wait until transfer finished
  while (!(*spi_status_register & SPI_SR_TXE));

  // return data read
  while ((*spi_status_register & SPI_SR_RXNE))
  {
    recv_data = *spi_data_register;
  }

  // wait until transfer finished
  while ((*spi_status_register & SPI_SR_BSY));

  return recv_data;
}

