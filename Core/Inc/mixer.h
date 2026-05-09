#ifndef __MIXER_H__
#define __MIXER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * X-frame motor mixer — combines a throttle base with three axis commands
 * (roll, pitch, yaw) into four motor pulse widths in microseconds.
 *
 * Frame layout (from pwm_esc.h, viewed from above, front = up):
 *
 *       Front
 *   M3(CCW)  M1(CW)
 *      \      /
 *       [body]
 *      /      \
 *   M2(CW)  M4(CCW)
 *       Rear
 *
 * Sign convention (right-hand rule, body frame):
 *   roll_cmd  > 0  →  roll right  (drone banks to its right side)
 *   pitch_cmd > 0  →  pitch up    (nose climbs)
 *   yaw_cmd   > 0  →  yaw right   (nose rotates clockwise viewed from above)
 *
 * Mixing equations (X-frame):
 *   M1 (FR, CW)  = T - roll - pitch - yaw
 *   M2 (RL, CW)  = T + roll + pitch - yaw
 *   M3 (FL, CCW) = T + roll - pitch + yaw
 *   M4 (RR, CCW) = T - roll + pitch + yaw
 *
 * Yaw signs assume CW motors produce CCW body torque and vice versa, so
 * boosting the CCW pair yaws the body clockwise. Verify on a tethered
 * bench test before flight — flip yaw signs if the drone yaws backwards.
 */

/*
 * Compute four motor outputs and clamp each to [min_us, max_us].
 *
 *   throttle_us : base pulse width, typically 1000..2000
 *   roll_cmd    : signed command in µs (PID output, e.g. -300..+300)
 *   pitch_cmd   : same
 *   yaw_cmd     : same
 *   min_us, max_us : output clamp range (use ESC_MIN_US / ESC_MAX_US)
 *   out_us[4]   : results, indexed M1..M4 matching pwm_esc.h
 */
void Mixer_Compute(float throttle_us,
                   float roll_cmd,
                   float pitch_cmd,
                   float yaw_cmd,
                   uint16_t min_us,
                   uint16_t max_us,
                   uint16_t out_us[4]);

#ifdef __cplusplus
}
#endif

#endif /* __MIXER_H__ */
