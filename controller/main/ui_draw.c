#include "ui_draw.h"
#include "rc_protocol.h"
#include <stdio.h>

#define ARM_HOLD_MS 2000u   /* drone side: arming.h ARM_HOLD_MS */

enum { ARM_DISARMED = 0, ARM_ARMED = 1, ARM_ARMING = 2 };

/* Stick box, top-right. */
#define BOX_X  102
#define BOX_Y  18
#define BOX_S  26

static bool blink(const UiState *s)
{
    return (s->now_ms / 400u) % 2u == 0u;
}

static void header(Gfx *g, const UiState *s)
{
    const char *title;
    bool inverted = false;

    if (!s->radio_ok) {
        title = "NO RADIO";
    } else if (!s->drone_link) {
        title = "NO LINK";
    } else if (s->arm_state == ARM_ARMED) {
        title = "ARMED";
        inverted = true;
    } else if (s->arm_state == ARM_ARMING) {
        title = "ARMING";
    } else {
        title = "DISARMED";
    }

    if (inverted) {
        gfx_fill_rect(g, 0, 0, GFX_W, 16, true);
    }
    gfx_text(g, 1, 1, title, 2, !inverted);

    if (s->radio_ok && s->drone_link) {
        const char *mode = s->mode == 1 ? "ANGL" : "RATE";
        gfx_text(g, GFX_W - gfx_text_width(mode, 1), 1, mode, 1, !inverted);
    }
}

static void stick_box(Gfx *g, const UiState *s)
{
    gfx_rect(g, BOX_X, BOX_Y, BOX_S, BOX_S, true);
    const int c = BOX_S / 2;
    gfx_pixel(g, BOX_X + c, BOX_Y + c, true);              /* center mark */

    /* + roll = right; + pitch = stick back (toward you) = down on screen. */
    const int r = c - 3;
    const int dx = s->roll  * r / RC_AXIS_MAX;
    const int dy = s->pitch * r / RC_AXIS_MAX;
    gfx_fill_rect(g, BOX_X + c + dx - 1, BOX_Y + c + dy - 1, 3, 3, true);
}

/* Bottom line: the one thing the pilot should know now. Inverted = warning. */
static void hint(Gfx *g, const UiState *s)
{
    const int y = 56;
    const char *text = NULL;
    bool warn = false;

    if (!s->radio_ok) {
        text = "RADIO NOT FOUND"; warn = true;
    } else if (!s->drone_link) {
        /* tx_pct > 0: our packets arrive (nRF24 ACK / bridge), but the
         * flight controller does not answer. */
        text = s->tx_pct > 0 ? "NO REPLY FROM DRONE" : "SEARCHING FOR DRONE";
    } else if (s->arm_state == ARM_ARMED) {
        text = "BUTTON = DISARM";
    } else if (s->arm_state == ARM_ARMING) {
        text = "KEEP DRONE STILL";
    } else if (s->throttle > RC_THROTTLE_ARM_MAX) {
        if (!blink(s)) return;
        text = "THROTTLE TO ZERO"; warn = true;
    } else if (s->button) {
        /* Hold progress — the drone arms at ARM_HOLD_MS. */
        gfx_text(g, 0, y, "ARM", 1, true);
        const uint32_t held = s->button_held_ms > ARM_HOLD_MS ? ARM_HOLD_MS : s->button_held_ms;
        gfx_bar(g, 22, y - 1, GFX_W - 22, 9, (int)held, (int)ARM_HOLD_MS);
        return;
    } else {
        text = "HOLD BTN 2s TO ARM";
    }

    if (warn) {
        gfx_fill_rect(g, 0, y - 1, gfx_text_width(text, 1) + 2, 9, true);
    }
    gfx_text(g, 1, y, text, 1, !warn);
}

void ui_render(Gfx *g, const UiState *s)
{
    char line[24];
    gfx_clear(g);
    header(g, s);

    /* Local slider: always meaningful, even without a link. */
    gfx_text(g, 0, 20, "THR", 1, true);
    gfx_bar(g, 20, 19, 52, 9, s->throttle, RC_THROTTLE_MAX);
    snprintf(line, sizeof(line), "%3u%%", (unsigned)(s->throttle * 100u / RC_THROTTLE_MAX));
    gfx_text(g, 75, 20, line, 1, true);

    if (s->drone_link) {
        snprintf(line, sizeof(line), "OUT %uus", (unsigned)s->drone_throttle_us);
    } else {
        snprintf(line, sizeof(line), "OUT ----");
    }
    gfx_text(g, 0, 32, line, 1, true);

    if (s->drone_link) {
        snprintf(line, sizeof(line), "TX %3u%% RX %3u%%", (unsigned)s->tx_pct, (unsigned)s->rx_pct);
    } else {
        snprintf(line, sizeof(line), "TX %3u%% RX  --", (unsigned)s->tx_pct);
    }
    gfx_text(g, 0, 44, line, 1, true);

    stick_box(g, s);
    hint(g, s);
}
