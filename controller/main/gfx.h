#ifndef CONTROLLER_GFX_H
#define CONTROLLER_GFX_H

/*
 * 128x64 monochrome framebuffer — pure C, no ESP-IDF (host-tested).
 *
 * Layout is the SSD1306's own: 8 pages of 128 bytes, each byte a vertical
 * strip of 8 pixels, bit 0 on top. oled_flush() sends it as-is.
 *
 * Drawing clips at the screen edges; any coordinates are safe.
 */

#include <stdbool.h>
#include <stdint.h>

#define GFX_W        128
#define GFX_H        64
#define GFX_BUF_SIZE (GFX_W * GFX_H / 8)

/* Text cell: 5x7 glyph + 1 px spacing. Scale multiplies both. */
#define GFX_CHAR_W   6
#define GFX_CHAR_H   8

typedef struct {
    uint8_t buf[GFX_BUF_SIZE];
} Gfx;

void gfx_clear(Gfx *g);
void gfx_pixel(Gfx *g, int x, int y, bool on);
bool gfx_get(const Gfx *g, int x, int y);

void gfx_fill_rect(Gfx *g, int x, int y, int w, int h, bool on);
void gfx_rect(Gfx *g, int x, int y, int w, int h, bool on);   /* 1 px outline */

/* Draw text with its top-left at (x, y). Characters outside the font draw
 * as '?'. Returns the x just past the last character. */
int  gfx_text(Gfx *g, int x, int y, const char *s, int scale, bool on);
int  gfx_text_width(const char *s, int scale);

/* Outlined horizontal bar filled to value/max. */
void gfx_bar(Gfx *g, int x, int y, int w, int h, int value, int max);

#endif /* CONTROLLER_GFX_H */
