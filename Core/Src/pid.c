#include "pid.h"

#define PID_TWO_PI 6.2831853f

static float clampf(float v, float limit)
{
    if (v >  limit) return  limit;
    if (v < -limit) return -limit;
    return v;
}

void PID_Init(PID_t *pid,
              float kp, float ki, float kd,
              float i_limit, float out_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_limit   = i_limit;
    pid->out_limit = out_limit;
    pid->d_lpf_hz  = 0.0f;

    PID_Reset(pid);
}

void PID_Reset(PID_t *pid)
{
    pid->integral         = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->has_prev         = 0u;
    pid->d_filt           = 0.0f;
    pid->p_term = pid->i_term = pid->d_term = pid->output = 0.0f;
}

void PID_ResetIntegral(PID_t *pid)
{
    pid->integral = 0.0f;
}

void PID_SetGains(PID_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_SetDLpf(PID_t *pid, float cutoff_hz)
{
    pid->d_lpf_hz = cutoff_hz > 0.0f ? cutoff_hz : 0.0f;
}

float PID_Update(PID_t *pid, float setpoint, float measurement, float dt)
{
    const float error = setpoint - measurement;

    /* Integral term with anti-windup clamp. */
    pid->integral += error * dt;
    pid->integral  = clampf(pid->integral, pid->i_limit);

    /* Derivative on measurement (negated — d(error)/dt = -d(measurement)/dt
     * when setpoint is constant, which avoids spikes on setpoint steps).
     * Skip on the first call so prev_measurement isn't garbage. */
    if (pid->has_prev && dt > 0.0f) {
        const float d_raw = (measurement - pid->prev_measurement) / dt;
        if (pid->d_lpf_hz > 0.0f) {
            /* First-order low-pass, discretized for this dt. */
            const float rc = 1.0f / (PID_TWO_PI * pid->d_lpf_hz);
            pid->d_filt += (dt / (rc + dt)) * (d_raw - pid->d_filt);
        } else {
            pid->d_filt = d_raw;
        }
    }
    const float d_meas = pid->d_filt;
    pid->prev_measurement = measurement;
    pid->has_prev = 1u;

    pid->p_term = pid->kp * error;
    pid->i_term = pid->ki * pid->integral;
    pid->d_term = -pid->kd * d_meas;
    pid->output = clampf(pid->p_term + pid->i_term + pid->d_term, pid->out_limit);

    return pid->output;
}
