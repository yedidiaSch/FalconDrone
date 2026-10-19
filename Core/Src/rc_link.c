#include "rc_link.h"
#include <string.h>

#define WINDOW_MS        1000u
#define EXPECTED_PER_WIN (WINDOW_MS / RC_PERIOD_MS)

void RcLink_Init(RcLink *l, uint32_t now_ms)
{
    memset(l, 0, sizeof(*l));
    l->state = RC_LINK_NONE;
    l->window_start_ms = now_ms;
}

void RcLink_OnPacket(RcLink *l, const RcPacket *p, uint32_t now_ms)
{
    l->last = *p;
    l->last_rx_ms = now_ms;
    l->state = RC_LINK_UP;
    if (l->window_count < UINT16_MAX) {
        l->window_count++;
    }
}

RcLinkState RcLink_Update(RcLink *l, uint32_t now_ms)
{
    const uint32_t in_window = now_ms - l->window_start_ms;
    if (in_window >= WINDOW_MS) {
        /* More than one window elapsed with no update → nothing arrived in
         * the most recent full window. */
        const uint32_t pct = (in_window >= 2u * WINDOW_MS)
                           ? 0u
                           : (uint32_t)l->window_count * 100u / EXPECTED_PER_WIN;
        l->link_pct = (uint8_t)(pct > 100u ? 100u : pct);
        l->window_count = 0;
        l->window_start_ms = now_ms;
    }

    if (l->state == RC_LINK_UP && (now_ms - l->last_rx_ms) > RC_LINK_TIMEOUT_MS) {
        l->state = RC_LINK_LOST;
        l->link_pct = 0;
    }
    return l->state;
}

float RcLink_Axis(int16_t v)
{
    if (v >  RC_AXIS_MAX) v =  RC_AXIS_MAX;
    if (v < -RC_AXIS_MAX) v = -RC_AXIS_MAX;
    return (float)v / (float)RC_AXIS_MAX;
}

float RcLink_Throttle(uint16_t v)
{
    if (v > RC_THROTTLE_MAX) v = RC_THROTTLE_MAX;
    return (float)v / (float)RC_THROTTLE_MAX;
}
