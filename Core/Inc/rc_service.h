#ifndef __RC_SERVICE_H__
#define __RC_SERVICE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "rc_link.h"
#include "rc_protocol.h"

/*
 * Drone end of the radio link: nRF24 in PRX mode on SPI2, polled from
 * ControlTask once per tick (100 Hz; the controller sends at 50 Hz).
 *
 * Each poll drains the RX FIFO, updates the link supervisor and queues an
 * RcAck so the controller's next packet gets fresh drone status back.
 *
 * A missing radio is not fatal: the drone keeps flying on the PC tether and
 * init is retried every 2 s, so a loose wire can be fixed without a reset.
 *
 * ControlTask only — no locking.
 */

/* First init attempt. Call >= 100 ms after power-up. Returns true if the
 * radio answered. */
bool RcService_Init(uint32_t now_ms);

/*
 * Run one step.
 *   status : arm_state, mode and throttle_us to report to the controller
 *            (seq and link_pct are filled in here)
 *   out    : newest packet, valid when the returned state is RC_LINK_UP
 */
RcLinkState RcService_Poll(uint32_t now_ms, const RcAck *status, RcPacket *out);

bool          RcService_RadioOk(void);
const RcLink *RcService_Link(void);

#ifdef __cplusplus
}
#endif

#endif /* __RC_SERVICE_H__ */
