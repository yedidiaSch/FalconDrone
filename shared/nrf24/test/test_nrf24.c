/*
 * Host tests for the nRF24 driver: two simulated chips joined by a fake
 * "air", so a PTX and a PRX driver instance talk to each other end to end.
 *
 *   make -C shared/nrf24/test
 *
 * The simulation models the register map, the 3-deep FIFOs, STATUS flags,
 * dynamic payload widths, ACK payloads and the FEATURE lock of old chips.
 * Radio timing is not modelled: a CE rising edge delivers (or fails) at once.
 */
#include "nrf24.h"
#include "../src/nrf24_regs.h"
#include <stdio.h>
#include <string.h>

/* ---- Simulated chip ------------------------------------------------------ */

typedef struct { uint8_t len; uint8_t data[33]; } Pkt;
typedef struct { Pkt p[3]; int n; } Fifo;

typedef struct Sim {
    uint8_t reg[0x20];
    uint8_t rx_addr0[5], tx_addr[5];
    Fifo tx, rx;
    int ce;
    int present;           /* 0 = no chip: MISO floats high */
    int feature_locked;    /* old nRF24L01 / clone: needs ACTIVATE */
    struct Sim *peer;
} Sim;

static void fifo_push(Fifo *f, const uint8_t *d, uint8_t len)
{
    if (f->n == 3) return;              /* chip ignores writes to a full FIFO */
    f->p[f->n].len = len;
    memcpy(f->p[f->n].data, d, len);
    f->n++;
}

static Pkt fifo_pop(Fifo *f)
{
    Pkt p = f->p[0];
    memmove(&f->p[0], &f->p[1], sizeof(Pkt) * 2);
    f->n--;
    return p;
}

static uint8_t sim_status(Sim *s)
{
    uint8_t st = s->reg[REG_STATUS] & STATUS_IRQ_ALL;
    st |= (uint8_t)((s->rx.n ? 0u : 7u) << 1);   /* RX_P_NO: 7 = empty, else pipe 0 */
    if (s->tx.n == 3) st |= STATUS_TX_FULL;
    return st;
}

static int is_ptx(Sim *s) { return (s->reg[REG_CONFIG] & (CONFIG_PWR_UP | CONFIG_PRIM_RX)) == CONFIG_PWR_UP; }
static int is_prx(Sim *s) { return (s->reg[REG_CONFIG] & (CONFIG_PWR_UP | CONFIG_PRIM_RX)) == (CONFIG_PWR_UP | CONFIG_PRIM_RX); }

static int link_ok(Sim *a, Sim *b)
{
    const uint8_t dpl = FEATURE_EN_DPL | FEATURE_EN_ACK_PAY;
    return b != NULL && b->ce && is_prx(b) &&
           a->reg[REG_RF_CH] == b->reg[REG_RF_CH] &&
           (a->reg[REG_RF_SETUP] & (RF_DR_LOW | RF_DR_HIGH)) == (b->reg[REG_RF_SETUP] & (RF_DR_LOW | RF_DR_HIGH)) &&
           memcmp(a->tx_addr, b->rx_addr0, 5) == 0 &&
           memcmp(a->tx_addr, a->rx_addr0, 5) == 0 &&            /* PTX must hear the ACK */
           (a->reg[REG_FEATURE] & dpl) == dpl && (b->reg[REG_FEATURE] & dpl) == dpl;
}

static void air_transmit(Sim *ptx)
{
    Sim *prx = ptx->peer;
    if (ptx->reg[REG_STATUS] & STATUS_MAX_RT) return;   /* chip stalls until MAX_RT is cleared */
    if (!link_ok(ptx, prx)) {
        uint8_t plos = ptx->reg[REG_OBSERVE_TX] >> 4;
        if (plos < 15) plos++;
        ptx->reg[REG_OBSERVE_TX] = (uint8_t)((plos << 4) | (ptx->reg[REG_SETUP_RETR] & 0x0F));
        ptx->reg[REG_STATUS] |= STATUS_MAX_RT;   /* payload stays in the TX FIFO */
        return;
    }
    Pkt p = fifo_pop(&ptx->tx);
    fifo_push(&prx->rx, p.data, p.len);
    prx->reg[REG_STATUS] |= STATUS_RX_DR;
    if (prx->tx.n) {                              /* ACK carries the queued payload */
        Pkt ack = fifo_pop(&prx->tx);
        fifo_push(&ptx->rx, ack.data, ack.len);
        ptx->reg[REG_STATUS] |= STATUS_RX_DR;
    }
    ptx->reg[REG_OBSERVE_TX] &= 0xF0;
    ptx->reg[REG_STATUS] |= STATUS_TX_DS;
}

static void sim_spi(void *ctx, const uint8_t *tx, uint8_t *rx, size_t len)
{
    Sim *s = ctx;
    if (!s->present) { memset(rx, 0xFF, len); return; }

    const uint8_t cmd = tx[0];
    const uint8_t n = (uint8_t)(len - 1);
    rx[0] = sim_status(s);
    memset(&rx[1], 0, n);

    if ((cmd & 0xE0) == CMD_R_REGISTER) {
        const uint8_t r = cmd & REG_MASK;
        if (r == REG_RX_ADDR_P0)       memcpy(&rx[1], s->rx_addr0, n);
        else if (r == REG_TX_ADDR)     memcpy(&rx[1], s->tx_addr, n);
        else if (r == REG_STATUS)      rx[1] = sim_status(s);
        else if (r == REG_FIFO_STATUS) rx[1] = (uint8_t)((s->rx.n ? 0 : FIFO_RX_EMPTY) |
                                                         (s->tx.n ? 0 : FIFO_TX_EMPTY) |
                                                         (s->tx.n == 3 ? FIFO_TX_FULL : 0));
        else                           rx[1] = s->reg[r];
    } else if ((cmd & 0xE0) == CMD_W_REGISTER) {
        const uint8_t r = cmd & REG_MASK;
        if (r == REG_STATUS)           s->reg[r] &= (uint8_t)~(tx[1] & STATUS_IRQ_ALL);
        else if (r == REG_RX_ADDR_P0)  memcpy(s->rx_addr0, &tx[1], 5);
        else if (r == REG_TX_ADDR)     memcpy(s->tx_addr, &tx[1], 5);
        else if (r == REG_FEATURE)     { if (!s->feature_locked) s->reg[r] = tx[1]; }
        else if (r == REG_RF_CH)       { s->reg[r] = tx[1]; s->reg[REG_OBSERVE_TX] &= 0x0F; }
        else                           s->reg[r] = tx[1];
    } else if (cmd == CMD_R_RX_PAYLOAD) {
        if (s->rx.n) { Pkt p = fifo_pop(&s->rx); memcpy(&rx[1], p.data, n < p.len ? n : p.len); }
    } else if (cmd == CMD_W_TX_PAYLOAD || cmd == CMD_W_ACK_PAYLOAD) {
        fifo_push(&s->tx, &tx[1], n);
    } else if (cmd == CMD_FLUSH_TX) {
        s->tx.n = 0;
    } else if (cmd == CMD_FLUSH_RX) {
        s->rx.n = 0;
    } else if (cmd == CMD_ACTIVATE && n == 1 && tx[1] == 0x73) {
        s->feature_locked = 0;
    } else if (cmd == CMD_R_RX_PL_WID) {
        rx[1] = s->rx.n ? s->rx.p[0].len : 0;
    }
}

static void sim_ce(void *ctx, bool high)
{
    Sim *s = ctx;
    const int rising = high && !s->ce;
    s->ce = high;
    if (rising && is_ptx(s) && s->tx.n) air_transmit(s);
}

static void sim_delay(void *ctx, uint32_t us) { (void)ctx; (void)us; }

/* ---- Test harness -------------------------------------------------------- */

static int g_fail, g_checks;
#define CHECK(c) do { g_checks++; if (!(c)) { g_fail++; printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); } } while (0)

static Sim sim_a, sim_b;     /* a = controller (PTX), b = drone (PRX) */
static Nrf24 ptx, prx;

static Nrf24Port port_of(Sim *s) { return (Nrf24Port){ sim_spi, sim_ce, sim_delay, s }; }

static Nrf24Config cfg(Nrf24Role role)
{
    return (Nrf24Config){ .role = role, .channel = 108, .rate = NRF24_RATE_1MBPS,
                          .power = NRF24_PA_MAX, .address = { 'F', 'A', 'L', 'C', 'N' },
                          .retries = 5, .retry_delay_us = 500 };
}

static void setup_pair(void)
{
    memset(&sim_a, 0, sizeof sim_a); memset(&sim_b, 0, sizeof sim_b);
    sim_a.present = sim_b.present = 1;
    sim_a.peer = &sim_b;
    Nrf24Port pa = port_of(&sim_a), pb = port_of(&sim_b);
    Nrf24Config ca = cfg(NRF24_ROLE_PTX), cb = cfg(NRF24_ROLE_PRX);
    CHECK(nrf24_init(&ptx, &pa, &ca) == NRF24_OK);
    CHECK(nrf24_init(&prx, &pb, &cb) == NRF24_OK);
}

static Nrf24TxResult send_and_poll(const void *d, uint8_t len, uint8_t *ack, uint8_t *ack_len)
{
    CHECK(nrf24_send(&ptx, d, len) == NRF24_OK);
    return nrf24_poll_tx(&ptx, ack, ack_len);
}

static void test_init_registers(void)
{
    setup_pair();
    CHECK(sim_a.reg[REG_CONFIG] == (CONFIG_EN_CRC | CONFIG_CRCO | CONFIG_PWR_UP));
    CHECK(sim_b.reg[REG_CONFIG] == (CONFIG_EN_CRC | CONFIG_CRCO | CONFIG_PWR_UP | CONFIG_PRIM_RX));
    CHECK(sim_a.reg[REG_SETUP_RETR] == 0x15);              /* 500 us, 5 retries */
    CHECK(sim_a.reg[REG_RF_SETUP] == 0x06);                /* 1 Mbps, 0 dBm     */
    CHECK(sim_a.reg[REG_RF_CH] == 108);
    CHECK(sim_a.reg[REG_FEATURE] == (FEATURE_EN_DPL | FEATURE_EN_ACK_PAY));
    CHECK(sim_a.reg[REG_DYNPD] == DYNPD_DPL_P0);
    CHECK(memcmp(sim_a.tx_addr, "FALCN", 5) == 0 && memcmp(sim_a.rx_addr0, "FALCN", 5) == 0);
    CHECK(sim_b.ce == 0);                                  /* PRX not listening yet */
}

static void test_init_errors(void)
{
    Sim s = { .present = 0 };
    Nrf24 d; Nrf24Port p = port_of(&s); Nrf24Config c = cfg(NRF24_ROLE_PTX);
    CHECK(nrf24_init(&d, &p, &c) == NRF24_ERR_NOT_FOUND);

    s.present = 1;
    c.channel = 126;              CHECK(nrf24_init(&d, &p, &c) == NRF24_ERR_ARG);
    c = cfg(NRF24_ROLE_PTX); c.retry_delay_us = 300; CHECK(nrf24_init(&d, &p, &c) == NRF24_ERR_ARG);
    c = cfg(NRF24_ROLE_PTX); c.retries = 16;         CHECK(nrf24_init(&d, &p, &c) == NRF24_ERR_ARG);
    c = cfg(NRF24_ROLE_PTX);      CHECK(nrf24_init(&d, NULL, &c) == NRF24_ERR_ARG);
}

static void test_feature_unlock(void)
{
    Sim s = { .present = 1, .feature_locked = 1 };
    Nrf24 d; Nrf24Port p = port_of(&s); Nrf24Config c = cfg(NRF24_ROLE_PTX);
    CHECK(nrf24_init(&d, &p, &c) == NRF24_OK);
    CHECK(s.reg[REG_FEATURE] == (FEATURE_EN_DPL | FEATURE_EN_ACK_PAY));
}

static void test_link_with_ack_payload(void)
{
    setup_pair();
    nrf24_start_listening(&prx);

    const uint8_t pkt[10] = { 1, 0, 0xE8, 0x03, 0x10, 0x00, 0xF0, 0xFF, 0, 0 };
    const uint8_t status1[6] = { 0, 1, 1, 100, 0x4C, 0x04 };
    CHECK(nrf24_set_ack_payload(&prx, status1, sizeof status1) == NRF24_OK);

    uint8_t ack[NRF24_MAX_PAYLOAD], ack_len = 99;
    CHECK(send_and_poll(pkt, sizeof pkt, ack, &ack_len) == NRF24_TX_OK);
    CHECK(ack_len == 6 && memcmp(ack, status1, 6) == 0);

    uint8_t rxbuf[NRF24_MAX_PAYLOAD];
    CHECK(nrf24_available(&prx));
    CHECK(nrf24_read(&prx, rxbuf) == 10 && memcmp(rxbuf, pkt, 10) == 0);
    CHECK(!nrf24_available(&prx));
    CHECK(nrf24_read(&prx, rxbuf) == 0);
    CHECK((sim_a.reg[REG_STATUS] & STATUS_IRQ_ALL) == 0);    /* flags cleared */
    CHECK((sim_b.reg[REG_STATUS] & STATUS_IRQ_ALL) == 0);

    /* No payload queued → plain ACK. */
    CHECK(send_and_poll(pkt, sizeof pkt, ack, &ack_len) == NRF24_TX_OK);
    CHECK(ack_len == 0);
    CHECK(nrf24_read(&prx, rxbuf) == 10);
}

static void test_ack_payload_is_freshest(void)
{
    setup_pair();
    nrf24_start_listening(&prx);
    const uint8_t old[2] = { 1, 1 }, fresh[2] = { 2, 2 }, pkt[3] = { 7, 8, 9 };
    nrf24_set_ack_payload(&prx, old, 2);
    nrf24_set_ack_payload(&prx, fresh, 2);
    CHECK(sim_b.tx.n == 1);

    uint8_t ack[NRF24_MAX_PAYLOAD], ack_len;
    CHECK(send_and_poll(pkt, 3, ack, &ack_len) == NRF24_TX_OK);
    CHECK(ack_len == 2 && ack[0] == 2);

    /* ACK payload ignored when the caller passes NULL — RX FIFO still drained. */
    nrf24_set_ack_payload(&prx, fresh, 2);
    CHECK(send_and_poll(pkt, 3, NULL, NULL) == NRF24_TX_OK);
    CHECK(sim_a.rx.n == 0);
}

static void test_link_lost_and_recovery(void)
{
    setup_pair();                         /* PRX powered but NOT listening */
    const uint8_t a[1] = { 0xA }, b[1] = { 0xB };

    CHECK(send_and_poll(a, 1, NULL, NULL) == NRF24_TX_FAILED);
    CHECK(sim_a.tx.n == 0);               /* failed packet flushed */
    uint8_t lost, retries;
    nrf24_observe_tx(&ptx, &lost, &retries);
    CHECK(lost == 1 && retries == 5);

    /* Newest wins: a second send replaces anything still queued. */
    nrf24_send(&ptx, a, 1);
    nrf24_send(&ptx, b, 1);
    CHECK(sim_a.tx.n == 1 && sim_a.tx.p[0].data[0] == 0xB);
    nrf24_poll_tx(&ptx, NULL, NULL);

    /* Fail again but never poll: MAX_RT stays set in the chip. The next
     * send must clear it itself, or the radio would stall for good. */
    nrf24_send(&ptx, a, 1);
    CHECK(sim_a.reg[REG_STATUS] & STATUS_MAX_RT);

    nrf24_start_listening(&prx);
    CHECK(send_and_poll(a, 1, NULL, NULL) == NRF24_TX_OK);
    uint8_t buf[NRF24_MAX_PAYLOAD];
    CHECK(nrf24_read(&prx, buf) == 1 && buf[0] == 0xA);
}

static void test_mismatched_channel_fails(void)
{
    setup_pair();
    nrf24_start_listening(&prx);
    nrf24_write_reg(&prx, REG_RF_CH, 76);
    const uint8_t a[1] = { 1 };
    CHECK(send_and_poll(a, 1, NULL, NULL) == NRF24_TX_FAILED);
}

static void test_corrupt_width_flushes(void)
{
    setup_pair();
    const uint8_t junk[33] = { 0 };
    fifo_push(&sim_b.rx, junk, 33);       /* width > 32 can happen on a glitch */
    fifo_push(&sim_b.rx, junk, 4);
    uint8_t buf[NRF24_MAX_PAYLOAD];
    CHECK(nrf24_read(&prx, buf) == 0);
    CHECK(sim_b.rx.n == 0);
}

static void test_role_and_length_guards(void)
{
    setup_pair();
    const uint8_t big[33] = { 0 }, one[1] = { 0 };
    CHECK(nrf24_send(&prx, one, 1) == NRF24_ERR_ARG);           /* PRX can't send */
    CHECK(nrf24_set_ack_payload(&ptx, one, 1) == NRF24_ERR_ARG); /* PTX has no ACK payload */
    CHECK(nrf24_send(&ptx, big, 33) == NRF24_ERR_ARG);
    CHECK(nrf24_send(&ptx, one, 0) == NRF24_ERR_ARG);
}

static void test_power_down(void)
{
    setup_pair();
    nrf24_start_listening(&prx);
    nrf24_power_down(&prx);
    CHECK(sim_b.ce == 0 && (sim_b.reg[REG_CONFIG] & CONFIG_PWR_UP) == 0);
    const uint8_t a[1] = { 1 };
    CHECK(send_and_poll(a, 1, NULL, NULL) == NRF24_TX_FAILED);
}

int main(void)
{
    test_init_registers();
    test_init_errors();
    test_feature_unlock();
    test_link_with_ack_payload();
    test_ack_payload_is_freshest();
    test_link_lost_and_recovery();
    test_mismatched_channel_fails();
    test_corrupt_width_flushes();
    test_role_and_length_guards();
    test_power_down();

    printf("%s: %d checks, %d failed\n", g_fail ? "FAILED" : "OK", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
