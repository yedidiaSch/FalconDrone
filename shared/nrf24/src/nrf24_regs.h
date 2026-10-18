#ifndef NRF24_REGS_H
#define NRF24_REGS_H

/* nRF24L01+ register map and SPI commands (Product Specification v1.0). */

/* Commands */
#define CMD_R_REGISTER     0x00u   /* | reg */
#define CMD_W_REGISTER     0x20u   /* | reg */
#define CMD_R_RX_PAYLOAD   0x61u
#define CMD_W_TX_PAYLOAD   0xA0u
#define CMD_FLUSH_TX       0xE1u
#define CMD_FLUSH_RX       0xE2u
#define CMD_ACTIVATE       0x50u   /* + 0x73: unlocks FEATURE on non-plus / some clones */
#define CMD_R_RX_PL_WID    0x60u
#define CMD_W_ACK_PAYLOAD  0xA8u   /* | pipe */
#define CMD_NOP            0xFFu
#define REG_MASK           0x1Fu

/* Registers */
#define REG_CONFIG         0x00u
#define REG_EN_AA          0x01u
#define REG_EN_RXADDR      0x02u
#define REG_SETUP_AW       0x03u
#define REG_SETUP_RETR     0x04u
#define REG_RF_CH          0x05u
#define REG_RF_SETUP       0x06u
#define REG_STATUS         0x07u
#define REG_OBSERVE_TX     0x08u
#define REG_RX_ADDR_P0     0x0Au
#define REG_TX_ADDR        0x10u
#define REG_FIFO_STATUS    0x17u
#define REG_DYNPD          0x1Cu
#define REG_FEATURE        0x1Du

/* CONFIG */
#define CONFIG_MASK_RX_DR  (1u << 6)
#define CONFIG_MASK_TX_DS  (1u << 5)
#define CONFIG_MASK_MAX_RT (1u << 4)
#define CONFIG_EN_CRC      (1u << 3)
#define CONFIG_CRCO        (1u << 2)   /* 1 = 2-byte CRC */
#define CONFIG_PWR_UP      (1u << 1)
#define CONFIG_PRIM_RX     (1u << 0)

/* STATUS */
#define STATUS_RX_DR       (1u << 6)
#define STATUS_TX_DS       (1u << 5)
#define STATUS_MAX_RT      (1u << 4)
#define STATUS_IRQ_ALL     (STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT)
#define STATUS_TX_FULL     (1u << 0)

/* RF_SETUP */
#define RF_DR_LOW          (1u << 5)
#define RF_DR_HIGH         (1u << 3)
#define RF_PWR_SHIFT       1u

/* FIFO_STATUS */
#define FIFO_TX_FULL       (1u << 5)
#define FIFO_TX_EMPTY      (1u << 4)
#define FIFO_RX_EMPTY      (1u << 0)

/* FEATURE / DYNPD */
#define FEATURE_EN_DPL     (1u << 2)
#define FEATURE_EN_ACK_PAY (1u << 1)
#define DYNPD_DPL_P0       (1u << 0)

/* Timing (datasheet) */
#define T_PD2STBY_US       5000u   /* power down → standby, worst case (ext. clock) */
#define T_HCE_US           15u     /* CE high pulse to start TX, min 10 us          */

#endif /* NRF24_REGS_H */
