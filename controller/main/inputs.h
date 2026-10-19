#ifndef INPUTS_H
#define INPUTS_H

/*
 * Throttle pot, joystick, stick button and the ARM / MODE switches →
 * RcPacket-ready values.
 *
 * The stick center is measured at boot: leave the stick alone while the
 * controller powers up. inputs_read() is 24 ADC conversions (~1 ms), so
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
    bool     arm;         /* ARM switch ON (debounced)        */
    bool     angle;       /* MODE switch on ANGLE (debounced) */
    int      raw_throttle, raw_x, raw_y;   /* for calibration */
    int      raw_arm, raw_mode;            /* GPIO levels     */
} Inputs;

esp_err_t inputs_init(void);
void      inputs_read(Inputs *in);

#endif /* INPUTS_H */
