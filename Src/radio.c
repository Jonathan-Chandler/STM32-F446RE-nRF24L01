#include "radio.h"
#include "stm32f4xx_hal.h"

extern SPI_HandleTypeDef hspi1;

void radio_configure(void)
{
  uint8_t write_buffer[6];
  uint8_t read_buffer[2];
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

  // channel to use (x25, 0x64)
  write_buffer[0] = (RADIO_W_REGISTER | RADIO_RF_CH);
  write_buffer[1] = 100;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, write_buffer, 2, 1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

  // radio rf power/rate (0x26, 0x0E)
  write_buffer[0] = (RADIO_W_REGISTER | RADIO_RF_SETUP);
  write_buffer[1] = (RADIO_RF_PWR_MASK | RADIO_RF_DR_MASK);      // 0dBm, 2mbps
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, write_buffer, 2, 1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  
  // radio auto-retransmit retries / time between retries (0x24, 0x1F)
  write_buffer[0] = (RADIO_W_REGISTER | RADIO_SETUP_RETR);
  write_buffer[1] = ((1 << RADIO_SETUP_RETR_ARD_OFFS) | RADIO_SETUP_RETR_ARC_MASK);      // 500uS between retries | 15 retries
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, write_buffer, 2, 1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  
  // address (0x2B, 0x01, 0x02, 0x03, 0x04, 0x00)
  write_buffer[0] = (RADIO_W_REGISTER | RADIO_ADDR_P1);
  write_buffer[1] = 1;
  write_buffer[2] = 2;
  write_buffer[3] = 3;
  write_buffer[4] = 4;
  write_buffer[5] = 0;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, write_buffer, sizeof(write_buffer), 1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

  // enable dynamic payload length on pipe 0/1 (0x3C, 0x03)
  write_buffer[0] = (RADIO_W_REGISTER | RADIO_DYNPD);
  write_buffer[1] = (RADIO_DYNPD_DPL_P0 | RADIO_DYNPD_DPL_P1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, write_buffer, 2, 1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

  // enable auto acknowledge pipe 0/1
  //  write_buffer[0] = (RADIO_W_REGISTER | RADIO_EN_AA);
  //  write_buffer[1] = (RADIO_EN_AA_P0 | RADIO_EN_AA_P1);
  //  spi_read_write(SPI_REGISTER_WRITE_ONLY, write_buffer, 2);

  // enable ack payload (0x3D, 0x07)
  write_buffer[0] = (RADIO_W_REGISTER | RADIO_FEATURE);
  write_buffer[1] = (RADIO_FEATURE_EN_DYN_ACK | RADIO_FEATURE_EN_ACK_PAY | RADIO_FEATURE_EN_DPL);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, write_buffer, 2, 1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

  // flush rx FIFO (0xE2)
  write_buffer[0] = RADIO_FLUSH_RX;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, write_buffer, 1, 1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

  // flush tx FIFO (0xE1)
  write_buffer[0] = RADIO_FLUSH_TX;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, write_buffer, 1, 1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  
  // clear interrupts 
  write_buffer[0] = (RADIO_R_REGISTER | RADIO_STATUS);
  read_buffer[0] = 0;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, write_buffer, read_buffer, 1, 1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

  write_buffer[0] = (RADIO_W_REGISTER | RADIO_STATUS);
  write_buffer[1] = read_buffer[0];
  write_buffer[1] |= (RADIO_RX_DR | RADIO_TX_DS | RADIO_MAX_RT); // |= 0111 0000
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, write_buffer, 2, 1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

  // power on, ignore all interrupts (0x20, 0x73)
  write_buffer[0] = (RADIO_W_REGISTER | RADIO_CONFIG);
  write_buffer[1] = (RADIO_PWR_UP | RADIO_PRIM_RX | RADIO_MASK_RX_DR | RADIO_MASK_TX_DS | RADIO_MASK_MAX_RT);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, write_buffer, 2, 1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

  // chip enable
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
}

// void radio_configure_tx(void)
// {
//   uint8_t statusData;
// 
//   // address (0x2B, 0x01, 0x02, 0x03, 0x04, 0x00)
//   // TX pipe address sets the destination radio for the data.
//   uint8_t address[5] = {1, 2, 3, 4, 0};
//   spi_read_write_command_data(SPI_REGISTER_WRITE_ONLY, (RADIO_W_REGISTER | RADIO_TX_ADDR), address, sizeof(address));
// 
//   // RX pipe 0 needs the same address to receive ACK packets from the destination
//   spi_read_write_command_data(SPI_REGISTER_WRITE_ONLY, (RADIO_W_REGISTER | RADIO_ADDR_P0), address, sizeof(address));
// 
//   // power on, ignore all interrupts (0x20, 0x73), remove RADIO_PRIM_RX for transmit mode
//   statusData = (RADIO_PWR_UP | RADIO_MASK_RX_DR | RADIO_MASK_TX_DS | RADIO_MASK_MAX_RT);
//   spi_read_write_command_data(SPI_REGISTER_WRITE_ONLY, (RADIO_W_REGISTER | RADIO_CONFIG), &statusData, sizeof(statusData));
// }
 
// get pipe # with rx waiting to be read
uint8_t radio_rx_waiting(void)
{
  uint8_t write_buffer = (RADIO_R_REGISTER | RADIO_STATUS);
  uint8_t read_buffer = 0;

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, &write_buffer, &read_buffer, 1, 1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

  return read_buffer;
}
 
// Determine length of data in the RX FIFO buffer and read it.
// data must have room for <= 32 bytes
void radio_recv(uint8_t *data)
{
  uint8_t write_buffer[2] = {0};
  uint8_t read_buffer[32] = {0};
  uint8_t dataLength = 0;

  // read RX data length, will be returned in buffer[1]
  write_buffer[0] = RADIO_R_RX_PL_WID;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, write_buffer, read_buffer, 1, 1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

  dataLength = read_buffer[0];

  // if data length is > 32 bytes, reset to 32 bytes
  if (dataLength > 32)
    dataLength = 32;

  // read rx data from SPI 
  write_buffer[0] = RADIO_R_RX_PAYLOAD;
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, write_buffer, read_buffer, dataLength, 1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

  // copy returned data in buffer to data
  for (size_t i = 0; i < dataLength; i++)
  {
    data[i] = read_buffer[i+1];
  }

  // read status data register to read_buffer[0]
  write_buffer[0] = (RADIO_R_REGISTER | RADIO_STATUS);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, write_buffer, read_buffer, 1, 1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);

  // clear RX ready status by writing (current_status | RADIO_RX_DR) to status register
  write_buffer[0] = (RADIO_W_REGISTER | RADIO_STATUS);
  write_buffer[1] = read_buffer[0] | (RADIO_RX_DR);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, write_buffer, 2, 1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
}
 
// // send data in 32 byte sections
// void radio_send(uint8_t *data, size_t length)
// {
//   enable_gpio(GPIO_A, GPIO_1);     // Chip Enable
// 
//   // transmit limited to 32 byte frames
//   for (size_t currentFrame = 0; currentFrame < length; currentFrame += 32)
//   {
//     size_t currentFrameSize = (length - currentFrame);
// 
//     if (currentFrameSize > 32)
//       currentFrameSize = 32;
// 
//     // check TX full before writing
//     uint8_t currentStatus = 0;
//     spi_read_write_command_data(SPI_REGISTER_READ_WRITE, RADIO_FIFO_STATUS, &currentStatus, sizeof(currentStatus));
// 
//     while (currentStatus & RADIO_TX_FULL)
//     {
//       for (size_t i = 0; i < 16000000; i++) 
//       {
//         __asm__("nop");
//       }
// 
//       delay();
//       spi_read_write_command_data(SPI_REGISTER_READ_WRITE, RADIO_FIFO_STATUS, &currentStatus, sizeof(currentStatus));
//     }
//    
//     spi_read_write_command_data(SPI_REGISTER_WRITE_ONLY, RADIO_W_TX_PAYLOAD, &data[currentFrame], currentFrameSize);
//   }
// 
//   enable_gpio(GPIO_A, GPIO_1);     // CE
// }
// 
