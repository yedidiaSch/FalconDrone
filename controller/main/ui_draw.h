#ifndef UI_DRAW_H
#define UI_DRAW_H

/*
 * Controller screen layout — pure C (host-tested, and rendered to images on
 * the PC by controller/test so the layout can be checked without hardware).
 *
 *   ┌────────────────────────────────┐
 *   │ DISARMED                  RATE │  state, big (inverted when ARMED)
 *   │ THR [██████····]  45%   ┌────┐ │  local slider
 *   │ OUT 1350us              │ ·  │ │  throttle the drone is commanding
 *   │ TX 100%  RX  98%        └────┘ │  link both ways, stick position box
 *   │ ARM SWITCH TO ARM              │  what to do next / what is wrong
 *   └────────────────────────────────┘
 */

#include <stdbool.h>
#include <stdint.h>
#include "gfx.h"

typedef struct {
    bool     radio_ok;          /* radio hardware initialized            */
    bool     drone_link;        /* fresh RcAck from the drone            */
    uint8_t  arm_state;         /* RcAck.arm_state (valid with link)     */
    uint8_t  mode;              /* RcAck.mode                            */
    uint16_t drone_throttle_us; /* RcAck.throttle_us                     */
    uint8_t  tx_pct;            /* packets acknowledged by the drone's
                                   radio (nRF24) or bridge (ESP-NOW), %  */
    uint8_t  rx_pct;            /* RcAck.link_pct                        */
    uint16_t throttle;          /* slider, 0..RC_THROTTLE_MAX            */
    int16_t  roll, pitch;       /* stick, -RC_AXIS_MAX..+RC_AXIS_MAX     */
    bool     arm_sw;            /* ARM switch ON                         */
    bool     angle_sw;          /* MODE switch on ANGLE                  */
    uint32_t arm_sw_on_ms;      /* how long the ARM switch has been ON   */
    uint32_t now_ms;            /* for blinking                          */
} UiState;

void ui_render(Gfx *g, const UiState *s);

#endif /* UI_DRAW_H */
