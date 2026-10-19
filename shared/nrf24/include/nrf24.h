#ifndef NRF24_H
#define NRF24_H

/*
 * Portable nRF24L01+ driver — shared by the ESP32 controller and the STM32
 * flight controller. No platform headers: each board supplies an Nrf24Port
 * (SPI transaction, CE pin, delay) and the driver does the rest.
 *
 * Link setup used by FalconDrone (see rc_protocol.h):
 *   - one pipe (pipe 0), same 5-byte address on both ends
 *   - auto-ACK with retries, 2-byte CRC
 *   - dynamic payload length + ACK payloads, so the receiver (drone) can
 *     return status inside the ACK without the transmitter switching to RX
 *
 * Roles:
 *   PTX (controller): nrf24_send() → poll nrf24_poll_tx() until done.
 *   PRX (drone):      nrf24_start_listening(), then nrf24_read() when
 *                     nrf24_available(); queue the next ACK payload with
 *                     nrf24_set_ack_payload().
 *
 * Not thread-safe: one task owns the device. The driver never blocks except
 * for the short datasheet delays inside nrf24_init() and nrf24_send().
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NRF24_MAX_PAYLOAD 32u
#define NRF24_ADDR_LEN    5u

/* ---- Board glue ---------------------------------------------------------- */
typedef struct {
    /* One SPI transaction: CSN low, exchange `len` bytes full-duplex
     * (mode 0, MSB first, <= 10 MHz), CSN high. `tx` and `rx` are never NULL. */
    void (*spi_transfer)(void *ctx, const uint8_t *tx, uint8_t *rx, size_t len);
    void (*ce_write)(void *ctx, bool high);
    void (*delay_us)(void *ctx, uint32_t us);
    void *ctx;
} Nrf24Port;

/* ---- Configuration ------------------------------------------------------- */
typedef enum {
    NRF24_ROLE_PTX = 0,   /* transmitter (controller) */
    NRF24_ROLE_PRX = 1,   /* receiver (drone)         */
} Nrf24Role;

typedef enum {
    NRF24_RATE_250KBPS = 0,
    NRF24_RATE_1MBPS,
    NRF24_RATE_2MBPS,
} Nrf24Rate;

typedef enum {
    NRF24_PA_MIN = 0,    /* -18 dBm */
    NRF24_PA_LOW,        /* -12 dBm */
    NRF24_PA_HIGH,       /*  -6 dBm */
    NRF24_PA_MAX,        /*   0 dBm */
} Nrf24Power;

typedef struct {
    Nrf24Role  role;
    uint8_t    channel;                 /* 0..125 → 2400 + channel MHz */
    Nrf24Rate  rate;
    Nrf24Power power;
    uint8_t    address[NRF24_ADDR_LEN];
    uint8_t    retries;                 /* auto-retransmit count, 0..15 */
    uint16_t   retry_delay_us;          /* 250..4000, step 250          */
} Nrf24Config;

typedef enum {
    NRF24_OK = 0,
    NRF24_ERR_NOT_FOUND,   /* no chip answering on SPI (wiring / power), or a
                            * clone without dynamic payloads / ACK payloads */
    NRF24_ERR_ARG,
} Nrf24Err;

typedef enum {
    NRF24_TX_PENDING = 0,  /* still sending / waiting for the ACK         */
    NRF24_TX_OK,           /* ACK received                                */
    NRF24_TX_FAILED,       /* no ACK after all retries — packet dropped   */
} Nrf24TxResult;

typedef struct {
    Nrf24Port port;
    Nrf24Role role;
} Nrf24;

/* Configure the chip and power it up (PTX: standby, PRX: not yet listening).
 * Call >= 100 ms after the module is powered (datasheet power-on reset). */
Nrf24Err nrf24_init(Nrf24 *dev, const Nrf24Port *port, const Nrf24Config *cfg);

/* ---- PTX ----------------------------------------------------------------- */

/* Load one packet and start sending it (pulses CE). Any previous packet that
 * is still unsent is discarded, so the newest command always wins. */
Nrf24Err nrf24_send(Nrf24 *dev, const void *data, uint8_t len);

/* Check the transfer started by nrf24_send(). On NRF24_TX_OK, an ACK payload
 * (if the receiver queued one) is copied into ack_buf and its length stored
 * in *ack_len (0 = plain ACK). ack_buf must hold NRF24_MAX_PAYLOAD bytes;
 * pass NULL to ignore ACK payloads. */
Nrf24TxResult nrf24_poll_tx(Nrf24 *dev, uint8_t *ack_buf, uint8_t *ack_len);

/* Lost packets since the last nrf24_init / channel write (0..15, saturates)
 * and retransmits used by the last packet. */
void nrf24_observe_tx(Nrf24 *dev, uint8_t *lost, uint8_t *retries);

/* ---- PRX ----------------------------------------------------------------- */

void nrf24_start_listening(Nrf24 *dev);
void nrf24_stop_listening(Nrf24 *dev);

/* True if at least one received packet is waiting. */
bool nrf24_available(Nrf24 *dev);

/* Pop one received packet into buf (NRF24_MAX_PAYLOAD bytes).
 * Returns its length, or 0 if nothing valid was waiting. */
uint8_t nrf24_read(Nrf24 *dev, void *buf);

/* Queue the payload sent with the ACK of the NEXT received packet. Replaces
 * any payload still queued, so the transmitter always gets fresh status. */
Nrf24Err nrf24_set_ack_payload(Nrf24 *dev, const void *data, uint8_t len);

/* ---- Common -------------------------------------------------------------- */

void nrf24_power_down(Nrf24 *dev);

/* Raw register access, for bring-up and debugging. */
uint8_t nrf24_read_reg(Nrf24 *dev, uint8_t reg);
void    nrf24_write_reg(Nrf24 *dev, uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* NRF24_H */
