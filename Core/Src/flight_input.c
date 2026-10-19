#include "flight_input.h"
#include "flight_cfg.h"
#include "pwm_esc.h"

static float s_target_us = (float)ESC_MIN_US;
static float s_current_us = (float)ESC_MIN_US;

static float clamp_throttle(float us)
{
    if (us < (float)ESC_MIN_US) return (float)ESC_MIN_US;
    if (us > THROTTLE_MAX_US)   return THROTTLE_MAX_US;
    return us;
}

void FlightInput_Reset(void)
{
    s_target_us  = (float)ESC_MIN_US;
    s_current_us = (float)ESC_MIN_US;
}

void FlightInput_SetThrottle(float target_us)
{
    s_target_us = clamp_throttle(target_us);
}

void FlightInput_NudgeThrottle(float delta_us)
{
    s_target_us = clamp_throttle(s_target_us + delta_us);
}

FlightCmd FlightInput_Get(float dt)
{
    const float step = THROTTLE_SLEW_US_S * dt;
    const float diff = s_target_us - s_current_us;

    if (diff > step)       s_current_us += step;
    else if (diff < -step) s_current_us -= step;
    else                   s_current_us = s_target_us;

    const FlightCmd cmd = {
        .throttle_us = s_current_us,
        .roll_deg    = 0.0f,
        .pitch_deg   = 0.0f,
        .yaw_dps     = 0.0f,
    };
    return cmd;
}
