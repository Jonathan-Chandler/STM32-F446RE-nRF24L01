#ifndef INCLUDE_SPI_H
#define INCLUDE_SPI_H
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "rcc.h"
#include "gpio.h"

// SPI configuration
#define SPI_1_BASE    0x40013000        // SPI 1 base address
#define SPI_2_BASE    0x40003800        // SPI 1 base address
#define SPI_3_BASE    0x40003C00        // SPI 1 base address
#define SPI_4_BASE    0x40013400        // SPI 4 base address

#define SPI_CR1       0x00
#define SPI_DFF       (1 << 11)         // data frame format: 0=8-bit; 1 = 16-bit
#define SPI_CR1_SSM   (1 << 9)          // software slave management enable
#define SPI_CR1_SSI   (1 << 8)          // value to use on NSS pin for slave select with SSM
#define SPI_LSB_FIRST (1 << 7)
#define SPI_CR1_SPE   (1 << 6)          // enable spi
#define SPI_BAUD_OFFS 3                 // bits 5:3
#define SPI_BAUD_MASK (0x7 << 3)        // 0x0=div 2; 0x1=div4; 0x2=div8; 0x3=div16; 0x4=div32; 0x5=div64; 0x6=div128; 0x7 = div 256
#define SPI_MASTER    (1 << 2)
#define SPI_CPOL_HIGH (1 << 1)          // clock idle high
#define SPI_CPOL_LOW  (0 << 1)          // clock idle low

#define SPI_CR2       0x04
#define SPI_CR2_SSOE  (1 << 2)          // 0 = output disabled in output master; 1 = output enabled in master mode

#define SPI_SR            0x08
#define SPI_SR_BSY        (1 << 7)
#define SPI_SR_OVR        (1 << 6)
#define SPI_SR_MODF       (1 << 5)
#define SPI_SR_CRCERR     (1 << 4)
#define SPI_SR_UDR        (1 << 3)
#define SPI_SR_CHSIDE     (1 << 2)
#define SPI_SR_TXE        (1 << 1)
#define SPI_SR_RXNE       (1 << 0)

#define SPI_TXE       (1 << 1)          // tx buffer empty
#define SPI_RXNE      (1 << 0)          // rx buffer not empty

#define SPI_DR        0x0C              // SPI data register (LSB for rx/tx in 8-bit mode)

#define SPI_CRCPR     0x10
#define SPI_RXCRCR    0x14
#define SPI_TXCRCR    0x18
#define SPI_I2SCFGR   0x1C
#define SPI_I2SPR     0x20

// spi commands
#define SPI_REGISTER_WRITE_ONLY 0
#define SPI_REGISTER_READ_WRITE 1

void configure_spi(void);

// void spi_read_write(bool read, volatile uint8_t *data, size_t size);
uint8_t spi_transfer(uint32_t *spi_register, uint8_t data);
void spi_read_write_command_data(bool read, uint8_t command, volatile uint8_t *data, size_t size);

#endif
