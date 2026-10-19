#include "rc_service.h"
#include "nrf24.h"
#include "nrf24_port_stm32.h"
#include "logger.h"
#include <string.h>

#define RETRY_INIT_MS  2000u
#define MAX_READS_POLL 3u      /* RX FIFO depth */

static Nrf24    s_radio;
static bool     s_radio_ok;
static bool     s_missing_logged;
static uint32_t s_last_init_ms;
static RcLink   s_link;

static bool try_init(uint32_t now_ms)
{
    static const Nrf24Config cfg = {
        .role           = NRF24_ROLE_PRX,
        .channel        = RC_CHANNEL,
        .rate           = NRF24_RATE_1MBPS,
        .power          = NRF24_PA_LOW,   /* bench: modules sit close together */
        .address        = RC_ADDRESS,
        .retries        = RC_RETRIES,
        .retry_delay_us = RC_RETRY_DELAY_US,
    };

    s_last_init_ms = now_ms;
    if (nrf24_init(&s_radio, Nrf24Port_Stm32(), &cfg) != NRF24_OK) {
        if (!s_missing_logged) {
            Log_Print("[RC] nRF24 not found — check wiring (PB1/PB12-15, 3V3). Retrying every 2 s\r\n");
            s_missing_logged = true;
        }
        return false;
    }
    nrf24_start_listening(&s_radio);
    Log_Printf("[RC] Radio ready on channel %u — waiting for the controller\r\n",
               (unsigned)RC_CHANNEL);
    return true;
}

bool RcService_Init(uint32_t now_ms)
{
    RcLink_Init(&s_link, now_ms);
    s_radio_ok = try_init(now_ms);
    return s_radio_ok;
}

static void log_transition(RcLinkState from, RcLinkState to)
{
    if (from == to) {
        return;
    }
    if (to == RC_LINK_UP) {
        Log_Print(from == RC_LINK_NONE ? "[RC] Link up\r\n" : "[RC] Link back\r\n");
    } else if (to == RC_LINK_LOST) {
        Log_Print("[RC] Link lost\r\n");
    }
}

RcLinkState RcService_Poll(uint32_t now_ms, const RcAck *status, RcPacket *out)
{
    if (!s_radio_ok) {
        if ((now_ms - s_last_init_ms) >= RETRY_INIT_MS) {
            s_radio_ok = try_init(now_ms);
        }
        return s_link.state;
    }

    const RcLinkState before = s_link.state;
    bool got = false;

    for (uint8_t i = 0; i < MAX_READS_POLL && nrf24_available(&s_radio); i++) {
        uint8_t buf[NRF24_MAX_PAYLOAD];
        const uint8_t len = nrf24_read(&s_radio, buf);
        if (len == sizeof(RcPacket)) {   /* anything else is not ours */
            RcPacket pkt;
            memcpy(&pkt, buf, sizeof(pkt));
            RcLink_OnPacket(&s_link, &pkt, now_ms);
            got = true;
        }
    }

    const RcLinkState state = RcLink_Update(&s_link, now_ms);
    log_transition(before, state);

    if (got) {
        RcAck ack = *status;
        ack.seq      = s_link.last.seq;
        ack.link_pct = s_link.link_pct;
        nrf24_set_ack_payload(&s_radio, &ack, sizeof(ack));
    }

    if (out != NULL) {
        *out = s_link.last;
    }
    return state;
}

bool RcService_RadioOk(void)
{
    return s_radio_ok;
}

const RcLink *RcService_Link(void)
{
    return &s_link;
}
