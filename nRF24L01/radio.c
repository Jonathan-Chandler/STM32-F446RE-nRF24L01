#include "radio.h"

void radio_configure(void)
{
  volatile uint8_t writeData[8];

  // channel to use (x25, 0x64)
  writeData[0] = (RADIO_W_REGISTER | RADIO_RF_CH);
  writeData[1] = (100);
  spi_read_write(SPI_REGISTER_WRITE_ONLY, writeData, 2);
  
  // radio rf power/rate (0x26, 0x0E)
  writeData[0] = (RADIO_W_REGISTER | RADIO_RF_SETUP);
  writeData[1] = (RADIO_RF_PWR_MASK | RADIO_RF_DR_MASK);      // 0dBm, 2mbps
  spi_read_write(SPI_REGISTER_WRITE_ONLY, writeData, 2);
  
  // radio auto-retransmit retries / time between retries (0x24, 0x1F)
  writeData[0] = (RADIO_W_REGISTER | RADIO_SETUP_RETR);
  writeData[1] = ((1 << RADIO_SETUP_RETR_ARD_OFFS) | RADIO_SETUP_RETR_ARC_MASK);      // 500uS between retries | 15 retries
  spi_read_write(SPI_REGISTER_WRITE_ONLY, writeData, 2);
  
  // address (0x2B, 0x01, 0x02, 0x03, 0x04, 0x00)
  uint8_t address[6] = {(RADIO_W_REGISTER | RADIO_ADDR_P1), 1, 2, 3, 4, 0};
  spi_read_write(SPI_REGISTER_WRITE_ONLY, address, sizeof(address));

  // enable dynamic payload length on pipe 0/1 (0x3C, 0x03)
  writeData[0] = (RADIO_W_REGISTER | RADIO_DYNPD);
  writeData[1] = (RADIO_DYNPD_DPL_P0 | RADIO_DYNPD_DPL_P1);
  spi_read_write(SPI_REGISTER_WRITE_ONLY, writeData, 2);

  // enable auto acknowledge pipe 0/1
//  writeData[0] = (RADIO_W_REGISTER | RADIO_EN_AA);
//  writeData[1] = (RADIO_EN_AA_P0 | RADIO_EN_AA_P1);
//  spi_read_write(SPI_REGISTER_WRITE_ONLY, writeData, 2);

  // enable ack payload (0x3D, 0x07)
  writeData[0] = (RADIO_W_REGISTER | RADIO_FEATURE);
  writeData[1] = (RADIO_FEATURE_EN_DYN_ACK | RADIO_FEATURE_EN_ACK_PAY | RADIO_FEATURE_EN_DPL);
  spi_read_write((RADIO_W_REGISTER | RADIO_FEATURE), writeData, 2);

  // flush rx FIFO (0xE2)
  writeData[0] = RADIO_FLUSH_RX;
  spi_read_write(SPI_REGISTER_WRITE_ONLY, writeData, 1);

  // flush tx FIFO (0xE1)
  writeData[0] = RADIO_FLUSH_TX;
  spi_read_write(SPI_REGISTER_WRITE_ONLY, writeData, 1);
  
  // clear interrupts 
  writeData[0] = (RADIO_R_REGISTER | RADIO_STATUS);
  spi_read_write(SPI_REGISTER_READ_WRITE, writeData, 2); // (0x07, 0x?X)
  writeData[0] = (RADIO_W_REGISTER | RADIO_STATUS);
  writeData[1] |= (RADIO_RX_DR | RADIO_TX_DS | RADIO_MAX_RT); // |= 0111 0000
  spi_read_write(SPI_REGISTER_WRITE_ONLY, writeData, 2); // (0x27, 0x7X) 

  // power on, ignore all interrupts (0x20, 0x73)
  writeData[0] = (RADIO_W_REGISTER | RADIO_CONFIG);
  writeData[1] = (RADIO_PWR_UP | RADIO_PRIM_RX | RADIO_MASK_RX_DR | RADIO_MASK_TX_DS | RADIO_MASK_MAX_RT);
  spi_read_write(SPI_REGISTER_WRITE_ONLY, writeData, 2);

  // chip enable
  enable_gpio(GPIO_A, GPIO_1);     // CE
}

// get pipe with rx waiting to be read
uint8_t radio_rx_waiting(void)
{
  uint8_t rxWaitingBuffer[2] = {0};
  rxWaitingBuffer[0] = (RADIO_R_REGISTER | RADIO_STATUS);
  rxWaitingBuffer[1] = ((rxWaitingBuffer[1] & RADIO_RX_P_NO_MASK) >> RADIO_RX_P_NO_OFFS);

  spi_read_write(SPI_REGISTER_READ_WRITE, rxWaitingBuffer, sizeof(rxWaitingBuffer));

  return rxWaitingBuffer[1];
}

// Determine length of data in the RX FIFO buffer and read it.
// data must have room for <= 32 bytes
void radio_recv(uint8_t *data)
{
  uint8_t dataLength = 0;
  volatile uint8_t dataBuffer[33] = {0}; // cmd byte + 32 bytes data

  // read RX data length, will be returned in buffer[1]
  dataBuffer[0] = RADIO_R_RX_PL_WID;
  spi_read_write(SPI_REGISTER_READ_WRITE, dataBuffer, 2);

  dataLength = dataBuffer[1];

  // if data length is > 4 bytes, reset to 32 bytes
  if (dataLength > 32)
    dataLength = 32;

  // read rx data from SPI 
  dataBuffer[0] = RADIO_R_RX_PAYLOAD;
  spi_read_write(SPI_REGISTER_READ_WRITE, dataBuffer, (dataLength + 1));

  // copy returned data in buffer to data
  for (size_t i = 0; i < dataLength; i++)
  {
    data[i] = dataBuffer[i+1];
  }

  // clear data received flag
  // current status is returned in dataBuffer[1]
  dataBuffer[0] = (RADIO_R_REGISTER | RADIO_STATUS);
  spi_read_write(SPI_REGISTER_READ_WRITE, dataBuffer, 2);

  // clear RX ready status by writing (current_status | RADIO_RX_DR) to status register
  dataBuffer[0] = (RADIO_W_REGISTER | RADIO_STATUS);
  dataBuffer[1] |= RADIO_RX_DR;
  spi_read_write(SPI_REGISTER_WRITE_ONLY, dataBuffer, 2);
}

