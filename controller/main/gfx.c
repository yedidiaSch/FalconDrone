#include "gfx.h"
#include "font5x7.h"
#include <string.h>

void gfx_clear(Gfx *g)
{
    memset(g->buf, 0, sizeof(g->buf));
}

void gfx_pixel(Gfx *g, int x, int y, bool on)
{
    if (x < 0 || x >= GFX_W || y < 0 || y >= GFX_H) {
        return;
    }
    uint8_t *b = &g->buf[(y / 8) * GFX_W + x];
    const uint8_t bit = (uint8_t)(1u << (y % 8));
    *b = on ? (uint8_t)(*b | bit) : (uint8_t)(*b & ~bit);
}

bool gfx_get(const Gfx *g, int x, int y)
{
    if (x < 0 || x >= GFX_W || y < 0 || y >= GFX_H) {
        return false;
    }
    return ((g->buf[(y / 8) * GFX_W + x] >> (y % 8)) & 1) != 0;
}

void gfx_fill_rect(Gfx *g, int x, int y, int w, int h, bool on)
{
    for (int j = y; j < y + h; j++) {
        for (int i = x; i < x + w; i++) {
            gfx_pixel(g, i, j, on);
        }
    }
}

void gfx_rect(Gfx *g, int x, int y, int w, int h, bool on)
{
    if (w <= 0 || h <= 0) {
        return;
    }
    gfx_fill_rect(g, x, y, w, 1, on);
    gfx_fill_rect(g, x, y + h - 1, w, 1, on);
    gfx_fill_rect(g, x, y, 1, h, on);
    gfx_fill_rect(g, x + w - 1, y, 1, h, on);
}

static void draw_char(Gfx *g, int x, int y, char c, int scale, bool on)
{
    unsigned char uc = (unsigned char)c;
    if (uc < FONT_FIRST || uc > FONT_LAST) {
        uc = '?';
    }
    const uint8_t *glyph = font5x7[uc - FONT_FIRST];
    for (int col = 0; col < FONT_W; col++) {
        for (int row = 0; row < FONT_H; row++) {
            if ((glyph[col] >> row) & 1u) {
                gfx_fill_rect(g, x + col * scale, y + row * scale, scale, scale, on);
            }
        }
    }
}

int gfx_text(Gfx *g, int x, int y, const char *s, int scale, bool on)
{
    for (; *s != '\0'; s++) {
        draw_char(g, x, y, *s, scale, on);
        x += GFX_CHAR_W * scale;
    }
    return x;
}

int gfx_text_width(const char *s, int scale)
{
    return (int)strlen(s) * GFX_CHAR_W * scale;
}

void gfx_bar(Gfx *g, int x, int y, int w, int h, int value, int max)
{
    gfx_rect(g, x, y, w, h, true);
    if (max <= 0 || w <= 4 || h <= 4) {
        return;
    }
    if (value < 0)   value = 0;
    if (value > max) value = max;
    const int inner = w - 4;
    gfx_fill_rect(g, x + 2, y + 2, inner * value / max, h - 4, true);
}
