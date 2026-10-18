#include "pid.h"

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

    PID_Reset(pid);
}

void PID_Reset(PID_t *pid)
{
    pid->integral         = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->has_prev         = 0u;
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

float PID_Update(PID_t *pid, float setpoint, float measurement, float dt)
{
    const float error = setpoint - measurement;

    /* Integral term with anti-windup clamp. */
    pid->integral += error * dt;
    pid->integral  = clampf(pid->integral, pid->i_limit);

    /* Derivative on measurement (negated — d(error)/dt = -d(measurement)/dt
     * when setpoint is constant, which avoids spikes on setpoint steps).
     * Skip on the first call so prev_measurement isn't garbage. */
    float d_meas = 0.0f;
    if (pid->has_prev && dt > 0.0f) {
        d_meas = (measurement - pid->prev_measurement) / dt;
    }
    pid->prev_measurement = measurement;
    pid->has_prev = 1u;

    const float output = pid->kp * error
                       + pid->ki * pid->integral
                       - pid->kd * d_meas;

    return clampf(output, pid->out_limit);
}
