#ifndef OLED_H
#define OLED_H

/*
 * SSD1306 128x64 over I2C (esp_lcd driver). Pins and address in board.h.
 *
 * A missing display is not fatal: oled_init() returns an error and the
 * controller keeps working without a screen.
 */

#include <stdint.h>
#include "esp_err.h"
#include "gfx.h"

/* Probes OLED_I2C_ADDR, then 0x3D (the other strap option). */
esp_err_t oled_init(void);

/* Send a full frame (~25 ms at 400 kHz). */
esp_err_t oled_flush(const Gfx *g);

#endif /* OLED_H */
