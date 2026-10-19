#ifndef INPUTS_H
#define INPUTS_H

/*
 * Slider, joystick and stick button → RcPacket-ready values.
 *
 * The stick center is measured at boot: leave the stick alone while the
 * controller powers up. inputs_read() is a dozen ADC conversions (~1 ms), so
 * it runs inside the radio task, right before each packet.
 */

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    uint16_t throttle;    /* 0 .. RC_THROTTLE_MAX            */
    int16_t  roll;        /* -RC_AXIS_MAX .. +RC_AXIS_MAX     */
    int16_t  pitch;
    bool     button;      /* debounced, true while held       */
    int      raw_throttle, raw_x, raw_y;   /* for calibration */
} Inputs;

esp_err_t inputs_init(void);
void      inputs_read(Inputs *in);

#endif /* INPUTS_H */
