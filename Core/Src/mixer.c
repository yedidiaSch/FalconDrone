#include "mixer.h"

static uint16_t clamp_u16(float v, uint16_t lo, uint16_t hi)
{
    if (v < (float)lo) return lo;
    if (v > (float)hi) return hi;
    return (uint16_t)v;
}

void Mixer_Compute(float throttle_us,
                   float roll_cmd,
                   float pitch_cmd,
                   float yaw_cmd,
                   uint16_t min_us,
                   uint16_t max_us,
                   uint16_t out_us[4])
{
    const float m1 = throttle_us - roll_cmd - pitch_cmd - yaw_cmd;
    const float m2 = throttle_us + roll_cmd + pitch_cmd - yaw_cmd;
    const float m3 = throttle_us + roll_cmd - pitch_cmd + yaw_cmd;
    const float m4 = throttle_us - roll_cmd + pitch_cmd + yaw_cmd;

    out_us[0] = clamp_u16(m1, min_us, max_us);
    out_us[1] = clamp_u16(m2, min_us, max_us);
    out_us[2] = clamp_u16(m3, min_us, max_us);
    out_us[3] = clamp_u16(m4, min_us, max_us);
}
