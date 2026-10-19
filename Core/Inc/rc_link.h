#ifndef __RC_LINK_H__
#define __RC_LINK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "rc_protocol.h"

/*
 * Radio link supervisor — pure logic, no hardware. rc_service.c feeds it the
 * packets read from the nRF24; ControlTask reads the state.
 *
 *   NONE ──first packet──▶ UP ──no packet for RC_LINK_TIMEOUT_MS──▶ LOST
 *                           ▲                                         │
 *                           └───────────── packet ◀───────────────────┘
 *
 * NONE means no controller has been heard since boot: the drone runs on the
 * PC tether ($t commands). UP → LOST while armed is the failsafe trigger.
 *
 * link_pct counts packets over 1 s windows against the expected
 * 1000 / RC_PERIOD_MS, so 100 % = no packet lost.
 */

typedef enum {
    RC_LINK_NONE = 0,
    RC_LINK_UP,
    RC_LINK_LOST,
} RcLinkState;

typedef struct {
    RcLinkState state;
    RcPacket    last;             /* newest packet (valid unless NONE) */
    uint32_t    last_rx_ms;
    uint32_t    window_start_ms;
    uint16_t    window_count;
    uint8_t     link_pct;
} RcLink;

void RcLink_Init(RcLink *l, uint32_t now_ms);

/* Record one received packet. */
void RcLink_OnPacket(RcLink *l, const RcPacket *p, uint32_t now_ms);

/* Advance timeouts and the link-quality window. Returns the new state. */
RcLinkState RcLink_Update(RcLink *l, uint32_t now_ms);

/* Packet fields → normalized values, clamped. */
float RcLink_Axis(int16_t v);        /* -1 .. +1 */
float RcLink_Throttle(uint16_t v);   /*  0 .. 1  */

#ifdef __cplusplus
}
#endif

#endif /* __RC_LINK_H__ */
