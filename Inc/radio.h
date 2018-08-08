#ifndef INCLUDE_RADIO_H
#define INCLUDE_RADIO_H
// #include <string.h>
#include <stdint.h>
// #include "stm32f4xx_hal_conf.h"
// 
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
#define RADIO_W_TX_PAYLOAD          0xA0    // low 3 bits = pipe to use; use in rx mode; transmit together with ack packet on pipe; max 3
#define RADIO_W_TX_PAYLOAD_NO_ACK   0xB0    // use in tx mode; disable autoack on this packet
#define RADIO_NOP                   0xFF    // no operation, can read status register

// register map for nRF24L01 (8 bit)
#define RADIO_CONFIG      0x00
#define RADIO_MASK_RX_DR  (1 << 6)      // 1=ignore IRQ; 0=rx dr on low interrupt IRQ
#define RADIO_MASK_TX_DS  (1 << 5)      // 1=ignore IRQ; 0=tx ds on low interrupt IRQ
#define RADIO_MASK_MAX_RT (1 << 4)      // 1=ignore IRQ; 0=reflect MAX_RT active on low
#define RADIO_EN_CRC      (1 << 3)      // enable CRC; forced if EN_AA is high
#define RADIO_CRCO        (1 << 2)      // CRC encoding scheme: 0=1 byte, 1=2 bytes
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

#define RADIO_STATUS          0x07
#define RADIO_RX_DR           (1 << 6)      // data ready on RX; write 1 to clear bit 0100 0000
#define RADIO_TX_DS           (1 << 5)      // data sent on TX; write 1 to clear bit
#define RADIO_MAX_RT          (1 << 4)      // maximum tx retransmits interrupt; write 1 to clear
#define RADIO_RX_P_NO_MASK    (0x7 << 1)    // rx pipe ready for rx_fifo: 0-5=pipe #; 6=unused; 7=rx fifo empty
#define RADIO_RX_P_NO_OFFS    1             // offset for rx pipe #
#define RADIO_TX_FULL         (1 << 0)      // 1=tx full; 0=available locations in tx fifo

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

#define RADIO_DYNPD               0x1C        // dynamic packet payload; requires en_dpl and enaa_p0
#define RADIO_DYNPD_DPL_P5        (1 << 5)    // pipe 5
#define RADIO_DYNPD_DPL_P4        (1 << 4)    // pipe 4
#define RADIO_DYNPD_DPL_P3        (1 << 3)    // pipe 3
#define RADIO_DYNPD_DPL_P2        (1 << 2)    // pipe 2
#define RADIO_DYNPD_DPL_P1        (1 << 1)    // pipe 1
#define RADIO_DYNPD_DPL_P0        (1 << 0)    // pipe 0

#define RADIO_FEATURE               0x1D
#define RADIO_FEATURE_EN_DPL        (1 << 2)   // enable dynamic payload length
#define RADIO_FEATURE_EN_ACK_PAY    (1 << 1)   // enable payload with ack; dynamic payload length must be en on pipe 0 ptx/prx
#define RADIO_FEATURE_EN_DYN_ACK    (1 << 0)   // enable W_TX_PAYLOAD_NOACK command
// 
void radio_configure(void);
// void radio_configure_tx(void);
void radio_recv(uint8_t *data);
// void radio_send(uint8_t *data, size_t length);
// uint8_t radio_get_erx_pipes(void);
uint8_t radio_rx_waiting(void);

#endif
