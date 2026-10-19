#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * Single-axis PID controller — pure math, no hardware or RTOS dependencies.
 *
 * Design choices:
 *   - Derivative on *measurement*, not error. Prevents the "derivative kick"
 *     when the setpoint changes step-wise (common with stick inputs).
 *   - Integral clamp (i_limit) for anti-windup. Without this, a saturated
 *     output would let the integral grow unbounded and overshoot badly when
 *     the error finally clears.
 *   - Output clamp (out_limit) is symmetric: ±out_limit. Saturation is also
 *     used to gate integration (clamping integral at i_limit is enough for
 *     basic flight; full back-calculation can come later if needed).
 *   - dt is passed per call so the same struct works at any loop rate and
 *     copes gracefully with jitter.
 *
 * Use one PID_t per axis (roll, pitch, yaw). Reset on disarm.
 */

typedef struct {
    float kp;
    float ki;
    float kd;

    float i_limit;    /* |integral|     <= i_limit   (anti-windup)   */
    float out_limit;  /* |output|       <= out_limit (saturation)    */

    float integral;
    float prev_measurement;
    uint8_t has_prev;  /* 0 on first call → skip derivative term     */

    /* Terms of the last PID_Update(), for the tuning log. */
    float p_term;
    float i_term;
    float d_term;
    float output;      /* clamped sum                                */
} PID_t;

void  PID_Init(PID_t *pid,
               float kp, float ki, float kd,
               float i_limit, float out_limit);

/* Zero internal state. Call when arming or after a long pause. */
void  PID_Reset(PID_t *pid);

/* Zero only the integral, keeping derivative history. Used to hold the
 * I-term at zero while the drone sits on the ground (no wind-up before
 * lift-off). */
void  PID_ResetIntegral(PID_t *pid);

/* Live gain update (e.g. from a tuning UI). Internal state is preserved. */
void  PID_SetGains(PID_t *pid, float kp, float ki, float kd);

/*
 * One control step.
 *   setpoint    : desired value (e.g. target roll rate, deg/s)
 *   measurement : current value from sensor (e.g. gyro x, deg/s)
 *   dt          : seconds since the previous call (e.g. 0.005f at 200 Hz)
 *
 * Returns the controller output, already clamped to ±out_limit.
 */
float PID_Update(PID_t *pid, float setpoint, float measurement, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
