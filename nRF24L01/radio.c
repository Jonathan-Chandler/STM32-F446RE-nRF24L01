#include "radio.h"

void radio_configure(void)
{
  volatile uint8_t writeData;

  // channel to use (x25, 0x64)
  writeData = 100;
  spi_read_write_command_data(SPI_REGISTER_WRITE_ONLY, (RADIO_W_REGISTER | RADIO_RF_CH), &writeData, sizeof(writeData));
  
  // radio rf power/rate (0x26, 0x0E)
  writeData = (RADIO_RF_PWR_MASK | RADIO_RF_DR_MASK);      // 0dBm, 2mbps
  spi_read_write_command_data(SPI_REGISTER_WRITE_ONLY, (RADIO_W_REGISTER | RADIO_RF_SETUP), &writeData, sizeof(writeData));
  
  // radio auto-retransmit retries / time between retries (0x24, 0x1F)
  writeData = ((1 << RADIO_SETUP_RETR_ARD_OFFS) | RADIO_SETUP_RETR_ARC_MASK);      // 500uS between retries | 15 retries
  spi_read_write_command_data(SPI_REGISTER_WRITE_ONLY, (RADIO_W_REGISTER | RADIO_SETUP_RETR), &writeData, sizeof(writeData));
  
  // address (0x2B, 0x01, 0x02, 0x03, 0x04, 0x00)
  uint8_t address[5] = {1, 2, 3, 4, 0};
  spi_read_write_command_data(SPI_REGISTER_WRITE_ONLY, (RADIO_W_REGISTER | RADIO_ADDR_P1), address, sizeof(address));

  // enable dynamic payload length on pipe 0/1 (0x3C, 0x03)
  writeData = (RADIO_DYNPD_DPL_P0 | RADIO_DYNPD_DPL_P1);
  spi_read_write_command_data(SPI_REGISTER_WRITE_ONLY, (RADIO_W_REGISTER | RADIO_DYNPD), &writeData, sizeof(writeData));

  // enable auto acknowledge pipe 0/1
//  writeData[0] = (RADIO_W_REGISTER | RADIO_EN_AA);
//  writeData[1] = (RADIO_EN_AA_P0 | RADIO_EN_AA_P1);
//  spi_read_write(SPI_REGISTER_WRITE_ONLY, writeData, 2);

  // enable ack payload (0x3D, 0x07)
  writeData = (RADIO_FEATURE_EN_DYN_ACK | RADIO_FEATURE_EN_ACK_PAY | RADIO_FEATURE_EN_DPL);
  spi_read_write_command_data(SPI_REGISTER_WRITE_ONLY, (RADIO_W_REGISTER | RADIO_FEATURE), &writeData, sizeof(writeData));

  // flush rx FIFO (0xE2)
  spi_read_write_command_data(SPI_REGISTER_WRITE_ONLY, RADIO_FLUSH_RX, NULL, 0);

  // flush tx FIFO (0xE1)
  spi_read_write_command_data(SPI_REGISTER_WRITE_ONLY, RADIO_FLUSH_TX, NULL, 0);
  
  // clear interrupts 
  spi_read_write_command_data(SPI_REGISTER_READ_WRITE, (RADIO_R_REGISTER | RADIO_STATUS), &writeData, sizeof(writeData)); // (0x07, 0x?X)
  writeData |= (RADIO_RX_DR | RADIO_TX_DS | RADIO_MAX_RT); // |= 0111 0000
  spi_read_write_command_data(SPI_REGISTER_WRITE_ONLY, (RADIO_W_REGISTER | RADIO_STATUS), &writeData, sizeof(writeData)); // (0x27, 0x7X) 

  // power on, ignore all interrupts (0x20, 0x73)
  writeData = (RADIO_PWR_UP | RADIO_PRIM_RX | RADIO_MASK_RX_DR | RADIO_MASK_TX_DS | RADIO_MASK_MAX_RT);
  spi_read_write_command_data(SPI_REGISTER_WRITE_ONLY, (RADIO_W_REGISTER | RADIO_CONFIG), &writeData, sizeof(writeData));

  // chip enable
  enable_gpio(GPIO_A, GPIO_1);     // CE
}

void radio_configure_tx(void)
{
  uint8_t statusData;

  // address (0x2B, 0x01, 0x02, 0x03, 0x04, 0x00)
  // TX pipe address sets the destination radio for the data.
  uint8_t address[5] = {1, 2, 3, 4, 0};
  spi_read_write_command_data(SPI_REGISTER_WRITE_ONLY, (RADIO_W_REGISTER | RADIO_TX_ADDR), address, sizeof(address));

  // RX pipe 0 needs the same address to receive ACK packets from the destination
  spi_read_write_command_data(SPI_REGISTER_WRITE_ONLY, (RADIO_W_REGISTER | RADIO_ADDR_P0), address, sizeof(address));

  // power on, ignore all interrupts (0x20, 0x73), remove RADIO_PRIM_RX for transmit mode
  statusData = (RADIO_PWR_UP | RADIO_MASK_RX_DR | RADIO_MASK_TX_DS | RADIO_MASK_MAX_RT);
  spi_read_write_command_data(SPI_REGISTER_WRITE_ONLY, (RADIO_W_REGISTER | RADIO_CONFIG), &statusData, sizeof(statusData));
}

// get pipe with rx waiting to be read
uint8_t radio_rx_waiting(void)
{
  uint8_t rxWaitingPipe = 0;

  spi_read_write_command_data(SPI_REGISTER_READ_WRITE, (RADIO_R_REGISTER | RADIO_STATUS), &rxWaitingPipe, sizeof(rxWaitingPipe));

  return rxWaitingPipe;
}

// Determine length of data in the RX FIFO buffer and read it.
// data must have room for <= 32 bytes
void radio_recv(uint8_t *data)
{
  volatile uint8_t dataBuffer[32] = {0}; // 32 bytes data
  uint8_t dataLength = 0;

  // read RX data length, will be returned in buffer[1]
  spi_read_write_command_data(SPI_REGISTER_READ_WRITE, RADIO_R_RX_PL_WID, dataBuffer, 1);

  dataLength = dataBuffer[0];

  // if data length is > 4 bytes, reset to 32 bytes
  if (dataLength > 32)
    dataLength = 32;

  // read rx data from SPI 
  spi_read_write_command_data(SPI_REGISTER_READ_WRITE, RADIO_R_RX_PAYLOAD, dataBuffer, dataLength);

  // copy returned data in buffer to data
  for (size_t i = 0; i < dataLength; i++)
  {
    data[i] = dataBuffer[i+1];
  }

  // read status data register to dataBuffer[0]
  spi_read_write_command_data(SPI_REGISTER_READ_WRITE, (RADIO_R_REGISTER | RADIO_STATUS), dataBuffer, 1);

  // clear RX ready status by writing (current_status | RADIO_RX_DR) to status register
  dataBuffer[0] |= RADIO_RX_DR;
  spi_read_write_command_data(SPI_REGISTER_WRITE_ONLY, (RADIO_W_REGISTER | RADIO_STATUS), dataBuffer, 1);
}

// send data in 32 byte sections
void radio_send(uint8_t *data, size_t length)
{
  enable_gpio(GPIO_A, GPIO_1);     // Chip Enable

  // transmit limited to 32 byte frames
  for (size_t currentFrame = 0; currentFrame < length; currentFrame += 32)
  {
    size_t currentFrameSize = (length - currentFrame);

    if (currentFrameSize > 32)
      currentFrameSize = 32;

    spi_read_write_command_data(SPI_REGISTER_WRITE_ONLY, RADIO_W_TX_PAYLOAD, &data[currentFrame], currentFrameSize);
  }

  enable_gpio(GPIO_A, GPIO_1);     // CE
}

