#include "nrf24.h"
#include "nrf24_regs.h"
#include <string.h>

/* ---- SPI primitives ------------------------------------------------------ */

/* One command + `len` data bytes. `out` may be NULL (data ignored), `in` may
 * be NULL (NOPs clocked out). Returns the STATUS byte the chip shifts out
 * while receiving the command. */
static uint8_t xfer(Nrf24 *dev, uint8_t cmd, const uint8_t *in, uint8_t *out, uint8_t len)
{
    uint8_t tx[1 + NRF24_MAX_PAYLOAD];
    uint8_t rx[1 + NRF24_MAX_PAYLOAD];

    tx[0] = cmd;
    if (in != NULL) {
        memcpy(&tx[1], in, len);
    } else {
        memset(&tx[1], CMD_NOP, len);
    }
    dev->port.spi_transfer(dev->port.ctx, tx, rx, (size_t)len + 1u);
    if (out != NULL) {
        memcpy(out, &rx[1], len);
    }
    return rx[0];
}

static uint8_t command(Nrf24 *dev, uint8_t cmd)
{
    return xfer(dev, cmd, NULL, NULL, 0);
}

static uint8_t status(Nrf24 *dev)
{
    return command(dev, CMD_NOP);
}

uint8_t nrf24_read_reg(Nrf24 *dev, uint8_t reg)
{
    uint8_t v;
    xfer(dev, CMD_R_REGISTER | (reg & REG_MASK), NULL, &v, 1);
    return v;
}

void nrf24_write_reg(Nrf24 *dev, uint8_t reg, uint8_t value)
{
    xfer(dev, CMD_W_REGISTER | (reg & REG_MASK), &value, NULL, 1);
}

static void write_addr(Nrf24 *dev, uint8_t reg, const uint8_t addr[NRF24_ADDR_LEN])
{
    xfer(dev, CMD_W_REGISTER | reg, addr, NULL, NRF24_ADDR_LEN);
}

/* STATUS interrupt flags are cleared by writing 1 to them. */
static void clear_flags(Nrf24 *dev, uint8_t flags)
{
    nrf24_write_reg(dev, REG_STATUS, flags & STATUS_IRQ_ALL);
}

/* ---- Init ---------------------------------------------------------------- */

static bool config_valid(const Nrf24Config *cfg)
{
    return cfg->channel <= 125u &&
           cfg->retries <= 15u &&
           cfg->retry_delay_us >= 250u && cfg->retry_delay_us <= 4000u &&
           (cfg->retry_delay_us % 250u) == 0u &&
           cfg->rate <= NRF24_RATE_2MBPS &&
           cfg->power <= NRF24_PA_MAX &&
           cfg->role <= NRF24_ROLE_PRX;
}

static uint8_t rf_setup_value(const Nrf24Config *cfg)
{
    uint8_t v = (uint8_t)(cfg->power << RF_PWR_SHIFT);
    if (cfg->rate == NRF24_RATE_250KBPS) v |= RF_DR_LOW;
    if (cfg->rate == NRF24_RATE_2MBPS)   v |= RF_DR_HIGH;
    return v;
}

/* DPL + ACK payloads live behind FEATURE. The nRF24L01+ has it unlocked;
 * the old nRF24L01 and some clones need the ACTIVATE command first. */
static bool enable_features(Nrf24 *dev)
{
    const uint8_t want = FEATURE_EN_DPL | FEATURE_EN_ACK_PAY;

    nrf24_write_reg(dev, REG_FEATURE, want);
    if (nrf24_read_reg(dev, REG_FEATURE) == want) {
        return true;
    }
    const uint8_t key = 0x73u;
    xfer(dev, CMD_ACTIVATE, &key, NULL, 1);
    nrf24_write_reg(dev, REG_FEATURE, want);
    return nrf24_read_reg(dev, REG_FEATURE) == want;
}

Nrf24Err nrf24_init(Nrf24 *dev, const Nrf24Port *port, const Nrf24Config *cfg)
{
    if (dev == NULL || port == NULL || cfg == NULL || !config_valid(cfg) ||
        port->spi_transfer == NULL || port->ce_write == NULL || port->delay_us == NULL) {
        return NRF24_ERR_ARG;
    }
    dev->port = *port;
    dev->role = cfg->role;

    dev->port.ce_write(dev->port.ctx, false);

    /* Configure while powered down. */
    nrf24_write_reg(dev, REG_CONFIG, CONFIG_EN_CRC | CONFIG_CRCO);

    /* Presence check: a missing or unpowered chip reads back 0x00 or 0xFF. */
    nrf24_write_reg(dev, REG_SETUP_AW, 0x03u);            /* 5-byte addresses */
    nrf24_write_reg(dev, REG_RF_CH, cfg->channel);
    if (nrf24_read_reg(dev, REG_SETUP_AW) != 0x03u ||
        nrf24_read_reg(dev, REG_RF_CH) != cfg->channel) {
        return NRF24_ERR_NOT_FOUND;
    }

    const uint8_t ard = (uint8_t)(cfg->retry_delay_us / 250u - 1u);
    nrf24_write_reg(dev, REG_SETUP_RETR, (uint8_t)((ard << 4) | cfg->retries));
    nrf24_write_reg(dev, REG_RF_SETUP, rf_setup_value(cfg));
    nrf24_write_reg(dev, REG_EN_AA, 0x01u);               /* auto-ACK on pipe 0 */
    nrf24_write_reg(dev, REG_EN_RXADDR, 0x01u);           /* pipe 0 only        */

    /* Pipe 0 receives the ACKs on the PTX side, so it must match TX_ADDR. */
    write_addr(dev, REG_RX_ADDR_P0, cfg->address);
    write_addr(dev, REG_TX_ADDR, cfg->address);

    if (!enable_features(dev)) {
        return NRF24_ERR_NOT_FOUND;
    }
    nrf24_write_reg(dev, REG_DYNPD, DYNPD_DPL_P0);

    command(dev, CMD_FLUSH_TX);
    command(dev, CMD_FLUSH_RX);
    clear_flags(dev, STATUS_IRQ_ALL);

    uint8_t config = CONFIG_EN_CRC | CONFIG_CRCO | CONFIG_PWR_UP;
    if (cfg->role == NRF24_ROLE_PRX) {
        config |= CONFIG_PRIM_RX;
    }
    nrf24_write_reg(dev, REG_CONFIG, config);
    dev->port.delay_us(dev->port.ctx, T_PD2STBY_US);

    return NRF24_OK;
}

/* ---- PTX ----------------------------------------------------------------- */

Nrf24Err nrf24_send(Nrf24 *dev, const void *data, uint8_t len)
{
    if (dev->role != NRF24_ROLE_PTX || data == NULL || len == 0u || len > NRF24_MAX_PAYLOAD) {
        return NRF24_ERR_ARG;
    }

    /* Newest command wins: drop anything unsent, and clear MAX_RT — while it
     * is set the chip refuses to transmit. */
    command(dev, CMD_FLUSH_TX);
    clear_flags(dev, STATUS_TX_DS | STATUS_MAX_RT);

    xfer(dev, CMD_W_TX_PAYLOAD, (const uint8_t *)data, NULL, len);

    /* A >= 10 us CE pulse sends one packet, including auto-retransmits;
     * the chip then returns to standby by itself. */
    dev->port.ce_write(dev->port.ctx, true);
    dev->port.delay_us(dev->port.ctx, T_HCE_US);
    dev->port.ce_write(dev->port.ctx, false);
    return NRF24_OK;
}

/* Read the ACK payload(s) that arrived with the ACK; keep the newest. */
static uint8_t drain_ack_payload(Nrf24 *dev, uint8_t *ack_buf)
{
    uint8_t len = 0;
    while ((nrf24_read_reg(dev, REG_FIFO_STATUS) & FIFO_RX_EMPTY) == 0u) {
        uint8_t width;
        xfer(dev, CMD_R_RX_PL_WID, NULL, &width, 1);
        if (width == 0u || width > NRF24_MAX_PAYLOAD) {
            command(dev, CMD_FLUSH_RX);   /* corrupt width — datasheet says flush */
            len = 0;
            break;
        }
        uint8_t scratch[NRF24_MAX_PAYLOAD];
        xfer(dev, CMD_R_RX_PAYLOAD, NULL, ack_buf != NULL ? ack_buf : scratch, width);
        len = width;
    }
    clear_flags(dev, STATUS_RX_DR);
    return ack_buf != NULL ? len : 0u;
}

Nrf24TxResult nrf24_poll_tx(Nrf24 *dev, uint8_t *ack_buf, uint8_t *ack_len)
{
    if (ack_len != NULL) {
        *ack_len = 0;
    }

    const uint8_t st = status(dev);

    if (st & STATUS_TX_DS) {
        const uint8_t len = drain_ack_payload(dev, ack_buf);
        if (ack_len != NULL) {
            *ack_len = len;
        }
        clear_flags(dev, STATUS_TX_DS);
        return NRF24_TX_OK;
    }
    if (st & STATUS_MAX_RT) {
        clear_flags(dev, STATUS_MAX_RT);
        command(dev, CMD_FLUSH_TX);
        return NRF24_TX_FAILED;
    }
    return NRF24_TX_PENDING;
}

void nrf24_observe_tx(Nrf24 *dev, uint8_t *lost, uint8_t *retries)
{
    const uint8_t v = nrf24_read_reg(dev, REG_OBSERVE_TX);
    if (lost != NULL)    *lost    = (uint8_t)(v >> 4);
    if (retries != NULL) *retries = (uint8_t)(v & 0x0Fu);
}

/* ---- PRX ----------------------------------------------------------------- */

void nrf24_start_listening(Nrf24 *dev)
{
    clear_flags(dev, STATUS_RX_DR);
    dev->port.ce_write(dev->port.ctx, true);
}

void nrf24_stop_listening(Nrf24 *dev)
{
    dev->port.ce_write(dev->port.ctx, false);
}

bool nrf24_available(Nrf24 *dev)
{
    return (nrf24_read_reg(dev, REG_FIFO_STATUS) & FIFO_RX_EMPTY) == 0u;
}

uint8_t nrf24_read(Nrf24 *dev, void *buf)
{
    if (buf == NULL || !nrf24_available(dev)) {
        return 0;
    }

    uint8_t width;
    xfer(dev, CMD_R_RX_PL_WID, NULL, &width, 1);
    if (width == 0u || width > NRF24_MAX_PAYLOAD) {
        command(dev, CMD_FLUSH_RX);
        clear_flags(dev, STATUS_RX_DR);
        return 0;
    }

    xfer(dev, CMD_R_RX_PAYLOAD, NULL, (uint8_t *)buf, width);
    clear_flags(dev, STATUS_RX_DR);
    return width;
}

Nrf24Err nrf24_set_ack_payload(Nrf24 *dev, const void *data, uint8_t len)
{
    if (dev->role != NRF24_ROLE_PRX || data == NULL || len == 0u || len > NRF24_MAX_PAYLOAD) {
        return NRF24_ERR_ARG;
    }
    /* Drop any stale status so the next ACK carries this one. */
    command(dev, CMD_FLUSH_TX);
    xfer(dev, CMD_W_ACK_PAYLOAD | 0u, (const uint8_t *)data, NULL, len);
    return NRF24_OK;
}

/* ---- Common -------------------------------------------------------------- */

void nrf24_power_down(Nrf24 *dev)
{
    dev->port.ce_write(dev->port.ctx, false);
    const uint8_t config = nrf24_read_reg(dev, REG_CONFIG);
    nrf24_write_reg(dev, REG_CONFIG, (uint8_t)(config & ~CONFIG_PWR_UP));
}
