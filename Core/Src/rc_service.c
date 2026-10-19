#include "rc_service.h"
#include "rc_radio.h"
#include "logger.h"
#include <stddef.h>

#define RETRY_INIT_MS  2000u
#define MAX_READS_POLL 4u      /* a 10 ms tick brings one packet, rarely two */

static bool     s_radio_ok;
static bool     s_missing_logged;
static uint32_t s_last_init_ms;
static RcLink   s_link;

static bool try_init(uint32_t now_ms)
{
    s_last_init_ms = now_ms;
    if (!RcRadio_Init()) {
        if (!s_missing_logged) {
            Log_Printf("[RC] %s. Retrying every 2 s\r\n", RcRadio_MissingHint());
            s_missing_logged = true;
        }
        return false;
    }
    Log_Printf("[RC] Radio ready (%s) — waiting for the controller\r\n", RcRadio_Name());
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

    RcPacket pkt;
    for (uint8_t i = 0; i < MAX_READS_POLL && RcRadio_Receive(&pkt); i++) {
        RcLink_OnPacket(&s_link, &pkt, now_ms);
        got = true;
    }

    const RcLinkState state = RcLink_Update(&s_link, now_ms);
    log_transition(before, state);

    if (got) {
        RcAck ack = *status;
        ack.seq      = s_link.last.seq;
        ack.link_pct = s_link.link_pct;
        RcRadio_Reply(&ack);
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
