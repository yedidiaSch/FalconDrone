/*
 * Host tests for the controller screen (gfx.c, ui_draw.c).
 *
 *   make -C controller/test              run the checks
 *   make -C controller/test screens      also write .pgm previews into screens/ (4x)
 *                                        to look at the layout without hardware
 */
#include "gfx.h"
#include "ui_draw.h"
#include "rc_protocol.h"
#include <stdio.h>
#include <string.h>

static int s_fail, s_checks;

#define CHECK(cond) do { s_checks++; if (!(cond)) { \
    s_fail++; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)

static int count_on(const Gfx *g, int x, int y, int w, int h)
{
    int n = 0;
    for (int j = y; j < y + h; j++)
        for (int i = x; i < x + w; i++)
            n += gfx_get(g, i, j);
    return n;
}

static void test_pixels_and_clipping(void)
{
    Gfx g;
    gfx_clear(&g);
    gfx_pixel(&g, 0, 0, true);
    gfx_pixel(&g, 127, 63, true);
    gfx_pixel(&g, 5, 9, true);
    CHECK(g.buf[0] == 0x01);                       /* page 0, bit 0 */
    CHECK(g.buf[7 * 128 + 127] == 0x80);           /* page 7, bit 7 */
    CHECK(g.buf[1 * 128 + 5] == 0x02);             /* y 9 = page 1, bit 1 */
    CHECK(gfx_get(&g, 5, 9) && !gfx_get(&g, 5, 8));
    gfx_pixel(&g, 5, 9, false);
    CHECK(!gfx_get(&g, 5, 9));

    gfx_clear(&g);                                  /* off-screen: no crash, no wrap */
    gfx_fill_rect(&g, -10, -10, 300, 300, true);
    CHECK(count_on(&g, 0, 0, GFX_W, GFX_H) == GFX_W * GFX_H);
    gfx_clear(&g);
    gfx_text(&g, 120, 60, "WWWW", 3, true);
    gfx_pixel(&g, -1, 0, true);
    gfx_pixel(&g, 128, 0, true);
    gfx_pixel(&g, 0, 64, true);
    CHECK(count_on(&g, 0, 0, 120, 60) == 0);
}

static void test_text(void)
{
    Gfx g;
    gfx_clear(&g);
    CHECK(gfx_text(&g, 0, 0, "AB", 1, true) == 12);
    CHECK(gfx_text_width("AB", 2) == 24);
    /* 'A' column 0 = 0x7E: rows 1..6 on, row 0 off. */
    CHECK(!gfx_get(&g, 0, 0) && gfx_get(&g, 0, 1) && gfx_get(&g, 0, 6));
    CHECK(count_on(&g, 5, 0, 1, 8) == 0);           /* spacing column empty */

    gfx_clear(&g);
    gfx_text(&g, 0, 0, "I", 2, true);               /* scale 2: 2x2 blocks */
    CHECK(count_on(&g, 0, 0, 12, 16) == 4 * (2 + 7 + 2));

    gfx_clear(&g);
    gfx_text(&g, 0, 0, "\x01", 1, true);            /* unknown → '?' */
    Gfx q;
    gfx_clear(&q);
    gfx_text(&q, 0, 0, "?", 1, true);
    CHECK(memcmp(g.buf, q.buf, sizeof(g.buf)) == 0);
}

static void test_bar(void)
{
    Gfx g;
    gfx_clear(&g);
    gfx_bar(&g, 0, 0, 54, 9, 0, 1000);
    const int outline = count_on(&g, 0, 0, 54, 9);
    CHECK(outline == 2 * 54 + 2 * 7);
    gfx_bar(&g, 0, 0, 54, 9, 500, 1000);
    CHECK(count_on(&g, 0, 0, 54, 9) == outline + 25 * 5);
    gfx_bar(&g, 0, 0, 54, 9, 5000, 1000);           /* clamps */
    CHECK(count_on(&g, 0, 0, 54, 9) == outline + 50 * 5);
}

static const UiState base = {
    .radio_ok = true, .drone_link = true, .arm_state = 0, .mode = 0,
    .drone_throttle_us = 1000, .tx_pct = 100, .rx_pct = 98,
};

static void test_ui_states(void)
{
    Gfx g;
    UiState s = base;

    s.arm_state = 1;                                /* ARMED: inverted header */
    ui_render(&g, &s);
    CHECK(count_on(&g, 0, 0, GFX_W, 16) > GFX_W * 16 / 2);

    s.arm_state = 0;                                /* DISARMED: mostly dark header */
    ui_render(&g, &s);
    CHECK(count_on(&g, 0, 0, GFX_W, 16) < GFX_W * 16 / 3);

    /* Stick dot moves right / down with roll / pitch. */
    s.roll = 0; s.pitch = 0;
    ui_render(&g, &s);
    Gfx c = g;
    s.roll = RC_AXIS_MAX; s.pitch = RC_AXIS_MAX;
    ui_render(&g, &s);
    CHECK(memcmp(c.buf, g.buf, sizeof(g.buf)) != 0);
    CHECK(count_on(&g, 102 + 20, 18 + 20, 4, 4) > 0);   /* bottom-right of box */
    CHECK(count_on(&c, 102 + 20, 18 + 20, 4, 4) == 0);

    /* Throttle warning blinks: on and off frames differ. */
    s = base;
    s.throttle = 400;
    s.now_ms = 0;
    ui_render(&g, &s);
    c = g;
    s.now_ms = 400;
    ui_render(&g, &s);
    CHECK(count_on(&c, 0, 55, GFX_W, 9) > count_on(&g, 0, 55, GFX_W, 9));

    /* ARM switch left ON without the drone arming: an inverted warning
     * replaces the plain hint once ARM_STUCK_MS has passed. */
    s = base;
    ui_render(&g, &s);
    const int plain = count_on(&g, 0, 55, GFX_W, 9);
    s.arm_sw = true; s.arm_sw_on_ms = 100;          /* just flipped: no warning yet */
    ui_render(&g, &s);
    CHECK(count_on(&g, 0, 55, GFX_W, 9) < 3 * plain);
    s.arm_sw_on_ms = 2000;
    ui_render(&g, &s);
    CHECK(count_on(&g, 0, 55, GFX_W, 9) > 3 * plain);

    /* Mode switch differs from the drone's mode: the mode text blinks. */
    s = base;                                       /* drone RATE */
    s.angle_sw = true;
    s.now_ms = 0;
    ui_render(&g, &s);
    const int on = count_on(&g, 100, 0, 28, 9);
    s.now_ms = 400;
    ui_render(&g, &s);
    CHECK(on > 0 && count_on(&g, 100, 0, 28, 9) == 0);
    s.angle_sw = false;                             /* agrees again: steady */
    ui_render(&g, &s);
    CHECK(count_on(&g, 100, 0, 28, 9) > 0);
}

static void write_pgm(const char *dir, const char *name, const Gfx *g)
{
    char path[256];
    snprintf(path, sizeof(path), "%s/%s.pgm", dir, name);
    FILE *f = fopen(path, "wb");
    if (f == NULL) {
        printf("cannot write %s\n", path);
        s_fail++;
        return;
    }
    const int k = 4, border = 8;
    const int W = GFX_W * k + 2 * border, H = GFX_H * k + 2 * border;
    fprintf(f, "P5\n%d %d\n255\n", W, H);
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            const int px = (x - border) / k, py = (y - border) / k;
            const int inside = x >= border && y >= border && px < GFX_W && py < GFX_H;
            unsigned char v = 40;                           /* bezel */
            if (inside) v = gfx_get(g, px, py) ? 235 : 0;   /* lit / dark */
            fputc(v, f);
        }
    }
    fclose(f);
}

static void write_screens(const char *dir)
{
    Gfx g;
    struct { const char *name; UiState s; } shots[] = {
        { "1_no_radio",     { .radio_ok = false, .throttle = 0 } },
        { "2_no_link",      { .radio_ok = true, .tx_pct = 0, .throttle = 120 } },
        { "3_disarmed",     base },
        { "4_throttle_up",  base },
        { "5_arm_sw_stuck", base },
        { "6_armed",        base },
        { "7_armed_angle",  base },
    };
    shots[3].s.throttle = 450;
    shots[4].s.arm_sw = true;  shots[4].s.arm_sw_on_ms = 2000;
    shots[5].s.arm_state = 1;  shots[5].s.throttle = 620; shots[5].s.drone_throttle_us = 1310;
    shots[5].s.arm_sw = true;
    shots[5].s.roll = 300;     shots[5].s.pitch = -700;
    shots[6].s = shots[5].s;   shots[6].s.mode = 1; shots[6].s.angle_sw = true;
    shots[6].s.rx_pct = 76;
    for (size_t i = 0; i < sizeof(shots) / sizeof(shots[0]); i++) {
        ui_render(&g, &shots[i].s);
        write_pgm(dir, shots[i].name, &g);
    }
    printf("screens written to %s/\n", dir);
}

int main(int argc, char **argv)
{
    test_pixels_and_clipping();
    test_text();
    test_bar();
    test_ui_states();
    if (argc > 1) {
        write_screens(argv[1]);
    }
    printf("%s: %d checks, %d failed\n", s_fail ? "FAIL" : "OK", s_checks, s_fail);
    return s_fail ? 1 : 0;
}
