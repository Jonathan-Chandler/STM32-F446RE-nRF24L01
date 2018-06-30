/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2014 Karl Palsson <karlp@tweak.net.au>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

// p57

// RCC
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
#define SPI_BAUD_MASK (0x7 << 3)        // 0x0=div 2; ...; 0x7 = div 256
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

// SCK default LOW
// commands for nRF24L01 (8 bit)
#define RADIO_R_REGISTER            0x00    // read command/status registers; 1-5 data bytes; low 5 bits=register map
#define RADIO_W_REGISTER            0x20    // write command/status registers; 1-5 data bytes; low 5 bits=register map
#define RADIO_R_RX_PAYLOAD          0x61    // read RX payload; LSB first; 1-32 data bytes (RX MODE)
#define RADIO_W_TX_PAYLOAD          0xA0    // read TX payload; LSB first; 1-32 data bytes
#define RADIO_FLUSH_TX              0xE1    // flush tx in tx mode
#define RADIO_FLUSH_RX              0xE2    // flush rx in rx mode
#define RADIO_REUSE_TX_PL           0xE3    // reuse last transmitted payload as long as CE is high; active until w_tx_payload/flush_tx
#define RADIO_ACTIVATE              0x50    // use in power down/standby only
#define RADIO_ACTIVATE_DATA         0x73    // R_RX_PL_WID + W_ACK_PAYLOAD + W_TX_PAYLOAD_NOACK
#define RADIO_R_RX_PL_WID           0x60    // read rx payload width
#define RADIO_W_ACK_PAYLOAD         0xA0    // low 3 bits = pipe to use; use in rx mode; transmit together with ack packet on pipe; max 3
#define RADIO_W_TX_PAYLOAD_NO_ACK   0xB0    // use in tx mode; disable autoack on this packet
#define RADIO_NOP                   0xFF    // no operation, can read status register

// register map for nRF24L01 (8 bit)
#define RADIO_CONFIG      0x00
#define RADIO_MASK_RX_DR  (1 << 6)      // 1=ignore IRQ; 0=rx dr on low interrupt IRQ
#define RADIO_MASK_RX_RT  (1 << 5)      // 1=ignore IRQ; 0=tx ds on low interrupt IRQ
#define RADIO_MASK_MAX_RT (1 << 4)      // 1=ignore IRQ; 0=reflect MAX_RT active on low
#define RADIO_PWR_UP      (1 << 1)      // power on/off
#define RADIO_PRIM_RX     (1 << 0)      // rx=1; tx=0

#define RADIO_EN_AA       0x01          // enable auto acknowledge
#define RADIO_EN_AA_P5    (1 << 5)      // enable auto acknowledge on pipe 5
#define RADIO_EN_AA_P4    (1 << 4)      // enable auto acknowledge on pipe 4
#define RADIO_EN_AA_P3    (1 << 3)      // enable auto acknowledge on pipe 3
#define RADIO_EN_AA_P2    (1 << 2)      // enable auto acknowledge on pipe 2
#define RADIO_EN_AA_P1    (1 << 1)      // enable auto acknowledge on pipe 1
#define RADIO_EN_AA_P0    (1 << 0)      // enable auto acknowledge on pipe 0


#define RADIO_EN_RXADDR   0x02
#define RADIO_EN_RX_P5    (1 << 5)      // enable data pipe 5
#define RADIO_EN_RX_P4    (1 << 4)      // enable data pipe 4
#define RADIO_EN_RX_P3    (1 << 3)      // enable data pipe 3
#define RADIO_EN_RX_P2    (1 << 2)      // enable data pipe 2
#define RADIO_EN_RX_P1    (1 << 1)      // enable data pipe 1
#define RADIO_EN_RX_P0    (1 << 0)      // enable data pipe 0

#define RADIO_SETUP_AW    0x03            // setup address widths
#define RADIO_SETUP_AW_3_BYTES 0x1        // 3 byte address width
#define RADIO_SETUP_AW_4_BYTES 0x2        // 4 byte address width
#define RADIO_SETUP_AW_5_BYTES 0x3        // 5 byte address width

#define RADIO_SETUP_RETR  0x04            // auto retransmit setup
#define RADIO_SETUP_RETR_ARD_MASK   0xF0  // auto retransmit delay
#define RADIO_SETUP_RETR_ARD_OFFS   4
#define RADIO_SETUP_RETR_ARC_MASK   0x0F  // auto retransmit count
#define RADIO_SETUP_RETR_ARC_OFFS   0

#define RADIO_RF_CH       0x05          // RF channel to use
#define RADIO_RF_CH_MASK  0x7F          // valid 0 - 125 (126/127 are invalid RF CH)
#define RADIO_RF_CH_OFFS  0             // valid 0 - 125 (126/127 are invalid RF CH)

#define RADIO_RF_SETUP    0x06
#define RADIO_RF_DR_MASK  (0x1 << 3)    // RF data rate: 0=1mbps; 1=2mbps
#define RADIO_RF_DR_OFFS  3             // RF data rate: 0=1mbps; 1=2mbps
#define RADIO_RF_PWR_MASK (0x03 << 1)   // RF TX mode Power: 00 = -18dbm; 01=-12dbm; 10=-6dbm; 11=0dbm;
#define RADIO_RF_PWR_OFFS 1             // RF TX mode Power: 00 = -18dbm; 01=-12dbm; 10=-6dbm; 11=0dbm;

#define RADIO_STATUS      0x07
#define RADIO_RX_DR       (1 << 6)      // data ready on RX; write 1 to clear bit
#define RADIO_TX_DS       (1 << 5)      // data sent on TX; write 1 to clear bit
#define RADIO_MAX_RT      (1 << 4)      // maximum tx retransmits interrupt; write 1 to clear
#define RADIO_RX_P_NO     (0x7 << 1)    // rx pipe ready for rx_fifo: 0-5=pipe #; 6=unused; 7=rx fifo empty
#define RADIO_TX_FULL     (1 << 0)      // 1=tx full; 0=available locations in tx fifo

#define RADIO_OBSERVE_TX  0x08
#define RADIO_OBSERVE_TXPLOS_CNT_MASK 0xF0  // count lost packets
#define RADIO_OBSERVE_TXPLOS_CNT_OFFS 4     // count lost packets
#define RADIO_OBSERVE_TXARC_CNT_MASK  0x0F  // count retransmitted packets
#define RADIO_OBSERVE_TXARC_CNT_OFFS  0     // count retransmitted packets

#define RADIO_CD          0x09          // carrier detect
#define RADIO_ADDR_P0     0x0A          // receive addres data pipe 0; 5-byte max-len; lsb first; byte count setup by setup_aw
#define RADIO_ADDR_P1     0x0B          // receive addres data pipe 1; 5-byte max-len; lsb first; byte count setup by setup_aw
#define RADIO_ADDR_P2     0x0C
#define RADIO_ADDR_P3     0x0D
#define RADIO_ADDR_P4     0x0E
#define RADIO_ADDR_P5     0x0F

#define RADIO_TX_ADDR     0x10
#define RADIO_RX_PW_P0    0x11          // number bytes ready in rx payload data pipe; 1-32 bytes
#define RADIO_RX_PW_P1    0x12          // number bytes ready in rx payload data pipe; 1-32 bytes
#define RADIO_RX_PW_P2    0x13          // number bytes ready in rx payload data pipe; 1-32 bytes
#define RADIO_RX_PW_P3    0x14          // number bytes ready in rx payload data pipe; 1-32 bytes
#define RADIO_RX_PW_P4    0x15          // number bytes ready in rx payload data pipe; 1-32 bytes
#define RADIO_RX_PW_P5    0x16          // number bytes ready in rx payload data pipe; 1-32 bytes

#define RADIO_FIFO_STATUS     0x17
#define RADIO_FIFO_TX_REUSE   (1 << 6)      // reuse last transmitted packet if set high; packet repeatedly resent while CE high
#define RADIO_FIFO_TX_FULL    (1 << 5)      // 1=tx fifo full; 0=locations in tx fifo available
#define RADIO_FIFO_TX_EMPTY   (1 << 4)      // 1=tx fifo empty; 0=data in tx fifo              
#define RADIO_FIFO_RX_FULL    (1 << 1)      // 1=rx fifo full; 0=locations in rx fifo available
#define RADIO_FIFO_RX_EMPTY   (1 << 0)      // 1=rx fifo empty; 0=data in rx fifo

#define RADIO_DYNPD       0x1C          // requires en_dpl and enaa_p0
#define RADIO_FEATURE     0x1D

// AHB1 = 0x4002 0000 -> 4002 FFFF
// AHB1 = 0x4002 0000 -> 4002 FFFF

void rcc_enable_gpio(uint32_t gpios);
void rcc_enable_spi(void);

void configure_gpio(uint32_t gpio_group, uint32_t gpio_pin, uint32_t gpio_mode, uint32_t gpio_pupd_mode);

void configure_spi(void);

void spi_send(uint16_t data);
//void spi_send2(uint8_t command, uint8_t *data, size_t size);
void spi_send2(uint8_t *data, size_t size);
void spi_transmit(volatile uint8_t *data, size_t size);

void init(void);

void enable_gpio(uint32_t gpio_group, uint32_t gpio_id);
void disable_gpio(uint32_t gpio_group, uint32_t gpio_id);
void toggle_gpio(uint32_t gpio_group, uint32_t gpio_id);

void radio_configure(void);
void radio_spi_transfer(uint8_t command, volatile uint8_t *data, size_t size);
uint8_t radio_get_erx_pipes(void);

int main(void)
{
  init();

  enable_gpio(GPIO_A, GPIO_0);      // slave select
  enable_gpio(GPIO_A, GPIO_1);      // chip enable

  radio_configure();

  while (1) 
  {
    uint32_t i = 0;

    for (i = 0; i < 1000000; i++) 
    {
      __asm__("nop");
    }

    uint8_t data = 0;
    radio_spi_transfer((RADIO_R_REGISTER | RADIO_CONFIG), &data, 1);
    if (data)
    {}
    //spi_send(RADIO_W_REGISTER | RADIO_CONFIG
    // data[0] = 2;
    // spi_send2(RADIO_W_REGISTER, data, 1);
//    spi_send2(RADIO_R_REGISTER, data, 1);
//
//    data[0] = 0;
//    spi_send2(RADIO_R_REGISTER, data, 1);
  }

  return 0;
}

void init(void)
{
  uint32_t enabled_gpio = RCC_GPIO_A | RCC_GPIO_B | RCC_GPIO_C;
  rcc_enable_gpio(enabled_gpio);
  rcc_enable_spi();

  configure_gpio(GPIO_A, GPIO_0, GPIO_MODE_OUT, GPIO_PUPD_PULL);      // slave select
  configure_gpio(GPIO_A, GPIO_1, GPIO_MODE_OUT, GPIO_PUPD_PULL);      // chip enable
//  configure_gpio(GPIO_C, GPIO_7, GPIO_MODE_OUT, GPIO_PUPD_PUSH);

  configure_spi();
}

void rcc_enable_gpio(uint32_t gpios)
{
  uint32_t *rcc_enable_offset = (uint32_t *)(RCC_BASE + RCC_AHB1ENR);
  
  *rcc_enable_offset |= gpios;
}

void rcc_enable_spi(void)
{
  uint32_t *rcc_enable_offset = (uint32_t *)(RCC_BASE + RCC_APB2ENR);
  
  *rcc_enable_offset |= (RCC_SPI_1_EN);
}

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

void configure_spi(void)
{
  uint32_t *spi_cr1_register = (uint32_t *)(SPI_1_BASE + SPI_CR1);
  uint32_t *spi_cr2_register = (uint32_t *)(SPI_1_BASE + SPI_CR2);
  uint32_t *gpio_afl_a_register = (uint32_t *)(GPIO_A + GPIO_AFRL);

  // CN5 (d10=3; d11=4; d12=5; d13=6)
  //configure_gpio(GPIO_B, GPIO_6, GPIO_MODE_ALT, GPIO_PUPD_NONE);    // D10 = CS
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

void spi_send(uint16_t data)
{
  uint32_t *spi_status_register = (uint32_t *)(SPI_1_BASE + SPI_SR);
  uint32_t *spi_data_register = (uint32_t *)(SPI_1_BASE + SPI_DR);
  uint32_t spi_status = *spi_status_register;

  disable_gpio(GPIO_A, GPIO_0);     // CSN

  /* Wait for transfer finished. */
  while (!(spi_status & SPI_SR_TXE));

  *spi_data_register = data;
  enable_gpio(GPIO_A, GPIO_0);     // CSN
}

//void spi_send2(uint8_t command, uint8_t *data, size_t size)
//{
//  volatile uint32_t *spi_status_register = (uint32_t *)(SPI_1_BASE + SPI_SR);
//  volatile uint32_t *spi_data_register = (uint32_t *)(SPI_1_BASE + SPI_DR);
//  uint32_t recv_data;
//
//  disable_gpio(GPIO_A, GPIO_0);     // CSN
//  disable_gpio(GPIO_A, GPIO_1);     // CSN
//
//
//  // wait until transfer finished, send command
//  while (!(*spi_status_register & SPI_SR_TXE));
//
//  // if rx not empty, read until rx buffer is empty
//  while ((*spi_status_register & SPI_SR_RXNE))
//  {
//    recv_data = *spi_data_register;
//  }
//  *spi_data_register = command;
//
//  // wait until transfer finished, send data
//  for (size_t currentByte = 0; currentByte < size; currentByte++)
//  {
//    // wait until tx empty
//    while (!(*spi_status_register & SPI_SR_TXE));
//
//    // if rx not empty, read until rx buffer is empty
//    while ((*spi_status_register & SPI_SR_RXNE))
//    {
//      recv_data = *spi_data_register;
//    }
//
//    *spi_data_register = data[currentByte];
//  }
//
//  // wait until tx empty
//  while (!(*spi_status_register & SPI_SR_TXE));
//
//  // if rx not empty, read until rx buffer is empty
//  while ((*spi_status_register & SPI_SR_RXNE))
//  {
//    //SPI_SR_BSY
//    //1. Wait until RXNE=1 to receive the last data.
//    //2. Wait until TXE=1 and then wait until BSY=0 before disabling the SPI.
//    //3. Read received data.
//    recv_data = *spi_data_register;
//  }
//
//  while ((*spi_status_register & SPI_SR_BSY));
//
//  if (recv_data)
//  {
//  }
//  enable_gpio(GPIO_A, GPIO_0);     // CSN
//  enable_gpio(GPIO_A, GPIO_1);     // CSN
//}
//
// 0000 1110 0000 1000
// 0E 08

void spi_send2(uint8_t *data, size_t size)
{
  volatile uint32_t *spi_status_register = (uint32_t *)(SPI_1_BASE + SPI_SR);
  volatile uint32_t *spi_data_register = (uint32_t *)(SPI_1_BASE + SPI_DR);
  uint32_t recv_data;

  disable_gpio(GPIO_A, GPIO_0);     // CSN
//  disable_gpio(GPIO_A, GPIO_1);     // CE

  // wait until transfer finished, send command
  while (!(*spi_status_register & SPI_SR_TXE));

  // if rx not empty, read until rx buffer is empty
  while ((*spi_status_register & SPI_SR_RXNE))
  {
    recv_data = *spi_data_register;
  }

  // wait until transfer finished, send data
  for (size_t currentByte = 0; currentByte < size; currentByte++)
  {
    // wait until tx empty
    while (!(*spi_status_register & SPI_SR_TXE));

    // if rx not empty, read until rx buffer is empty
    while ((*spi_status_register & SPI_SR_RXNE))
    {
      recv_data = *spi_data_register;
    }

    *spi_data_register = data[currentByte];
  }

  // wait until tx empty
  while (!(*spi_status_register & SPI_SR_TXE));

  // if rx not empty, read until rx buffer is empty
  while ((*spi_status_register & SPI_SR_RXNE))
  {
    //SPI_SR_BSY
    //1. Wait until RXNE=1 to receive the last data.
    //2. Wait until TXE=1 and then wait until BSY=0 before disabling the SPI.
    //3. Read received data.
    recv_data = *spi_data_register;
  }

  while ((*spi_status_register & SPI_SR_BSY));

  if (recv_data)
  {
  }
  enable_gpio(GPIO_A, GPIO_0);     // CSN
//  enable_gpio(GPIO_A, GPIO_1);     // CE
}

void spi_transmit(volatile uint8_t *data, size_t size)
{
  volatile uint32_t *spi_status_register = (uint32_t *)(SPI_1_BASE + SPI_SR);
  volatile uint32_t *spi_data_register = (uint32_t *)(SPI_1_BASE + SPI_DR);

  // wait until transfer finished, send command
  while (!(*spi_status_register & SPI_SR_TXE));

  // if rx not empty, read until rx buffer is empty
  while ((*spi_status_register & SPI_SR_RXNE))
  {
    // discard
    uint32_t recv_data = *spi_data_register;
    if (recv_data)
    {
    }
  }

  disable_gpio(GPIO_A, GPIO_0);     // CSN

  // wait until transfer finished, send data
  for (size_t currentByte = 0; currentByte < size; currentByte++)
  {
    *spi_data_register = data[currentByte];

    // wait until tx empty
    while (!(*spi_status_register & SPI_SR_TXE));

    // if rx not empty, replace data buffer with response
    while ((*spi_status_register & SPI_SR_RXNE))
    {
      data[currentByte] = *spi_data_register;
    }
  }

  while ((*spi_status_register & SPI_SR_BSY));

  enable_gpio(GPIO_A, GPIO_0);     // CSN
}

// 0000 1110 0111 0011
// 0E 73
void radio_configure(void)
{
  volatile uint8_t data[2];

  // power on, ignore all interrupts
  data[0] = (RADIO_W_REGISTER | RADIO_CONFIG);
  data[1] = (RADIO_PWR_UP | RADIO_PRIM_RX | RADIO_MASK_RX_DR | RADIO_MASK_RX_RT | RADIO_MASK_MAX_RT);
  spi_transmit(data, sizeof(data));

  // channel to use
  data[0] = (RADIO_W_REGISTER | RADIO_RF_CH);
  data[1] = (0);
  spi_transmit(data, sizeof(data));
  
  // radio rf power/rate
  data[0] = (RADIO_W_REGISTER | RADIO_RF_SETUP);
  data[1] = (RADIO_RF_PWR_MASK);      // 0dBm
  spi_transmit(data, sizeof(data));
  
  uint8_t address[5] = {1, 2, 3, 4, 0};
  radio_spi_transfer((RADIO_W_REGISTER | RADIO_ADDR_P1), address, sizeof(address));
}

void radio_spi_transfer(uint8_t command, volatile uint8_t *data, size_t size)
{
  volatile uint32_t *spi_status_register = (uint32_t *)(SPI_1_BASE + SPI_SR);
  volatile uint32_t *spi_data_register = (uint32_t *)(SPI_1_BASE + SPI_DR);

  // wait until transfer finished, send command
  while (!(*spi_status_register & SPI_SR_TXE));

  // if rx not empty, read until rx buffer is empty
  while ((*spi_status_register & SPI_SR_RXNE))
  {
    // discard
    uint32_t recv_data = *spi_data_register;
    if (recv_data)
    {
    }
  }

  disable_gpio(GPIO_A, GPIO_0);     // CSN

  // send command byte
  *spi_data_register = command;

  // wait until TX complete, discard contents of RX register
  while (!(*spi_status_register & SPI_SR_TXE));
  while ((*spi_status_register & SPI_SR_RXNE))
  {
    // discard response
    uint32_t recv_data = *spi_data_register;
    if (recv_data)
    {
    }
  }

  // send data bytes (LSB first)
  for (size_t currentByte = 0; currentByte < size; currentByte++)
  {
    *spi_data_register = data[currentByte];

    // wait until tx empty
    while (!(*spi_status_register & SPI_SR_TXE));

    // if rx not empty, replace data buffer with response
    while ((*spi_status_register & SPI_SR_RXNE))
    {
      data[currentByte] = *spi_data_register;
    }
  }

  while ((*spi_status_register & SPI_SR_BSY));

  enable_gpio(GPIO_A, GPIO_0);     // CSN
}

// void radio_read_register(uint8_t register_addr)
// {
//   volatile uint8_t data[2];
// 
//   // read config register
//   data[0] = (RADIO_R_REGISTER | register_addr);
//   data[1] = 0;
//   spi_transmit(data, sizeof(data));
// }

// void radio_powerup(void)
// {
//   uint8_t data[2];
//   // power on, ignore all interrupts
//   data[0] = (RADIO_PWR_UP | RADIO_PRIM_RX | RADIO_MASK_RX_DR | RADIO_MASK_RX_RT | RADIO_MASK_MAX_RT);
//   
//   data[1] = 0;
//   spi_send2(data, sizeof(data));
// }
// 
// uint8_t radio_get_erx_pipes(void)
// {
//   uint8_t result;
// }
