#include "flight_ctrl.h"
#include "flight_cfg.h"
#include "mixer.h"
#include "pwm_esc.h"
#include "logger.h"
#include <math.h>

#define TRIM_SAMPLES        200u   /* 2 s at 100 Hz */
#define TRIM_MAX_RATE_DPS     5.0f /* abort the capture if the board moves */

static PID_t        s_pid[LOOP_COUNT];
static FlightMode_t s_mode = FLIGHT_MODE_DEFAULT;

static float s_trim_roll  = TRIM_ROLL_DEG;
static float s_trim_pitch = TRIM_PITCH_DEG;

static uint8_t  s_trim_active;
static uint32_t s_trim_count;
static float    s_trim_sum_roll, s_trim_sum_pitch;

static void reset_all(void)
{
    for (int i = 0; i < LOOP_COUNT; i++) {
        PID_Reset(&s_pid[i]);
    }
}

void FlightCtrl_Init(void)
{
    PID_Init(&s_pid[LOOP_RATE_ROLL],  RATE_RP_KP,  RATE_RP_KI,  RATE_RP_KD, RATE_I_LIMIT, RATE_OUT_LIMIT);
    PID_Init(&s_pid[LOOP_RATE_PITCH], RATE_RP_KP,  RATE_RP_KI,  RATE_RP_KD, RATE_I_LIMIT, RATE_OUT_LIMIT);
    PID_Init(&s_pid[LOOP_RATE_YAW],   RATE_YAW_KP, RATE_YAW_KI, 0.0f,       RATE_I_LIMIT, RATE_OUT_LIMIT);

    /* Angle loop is P-only; its output is a rate setpoint in deg/s. */
    PID_Init(&s_pid[LOOP_ANGLE_ROLL],  ANGLE_KP, 0.0f, 0.0f, 0.0f, ANGLE_MAX_RATE_DPS);
    PID_Init(&s_pid[LOOP_ANGLE_PITCH], ANGLE_KP, 0.0f, 0.0f, 0.0f, ANGLE_MAX_RATE_DPS);
}

void FlightCtrl_Update(const IMUData *imu, const FlightCmd *cmd, float dt,
                       uint16_t motors[4], FlightCtrlDebug *dbg)
{
    float roll_rate_sp  = 0.0f;
    float pitch_rate_sp = 0.0f;

    if (s_mode == FLIGHT_MODE_ANGLE) {
        roll_rate_sp  = PID_Update(&s_pid[LOOP_ANGLE_ROLL],  cmd->roll_deg,
                                   imu->roll  - s_trim_roll,  dt);
        pitch_rate_sp = PID_Update(&s_pid[LOOP_ANGLE_PITCH], cmd->pitch_deg,
                                   imu->pitch - s_trim_pitch, dt);
    }

    const float roll_out  = PID_Update(&s_pid[LOOP_RATE_ROLL],  roll_rate_sp,  imu->gyro_x, dt);
    const float pitch_out = PID_Update(&s_pid[LOOP_RATE_PITCH], pitch_rate_sp, imu->gyro_y, dt);
    const float yaw_out   = PID_Update(&s_pid[LOOP_RATE_YAW],   cmd->yaw_dps,  imu->gyro_z, dt);

    /* On the ground: hold the I-terms at zero (no wind-up before lift-off). */
    if (cmd->throttle_us < LIFTOFF_US) {
        PID_ResetIntegral(&s_pid[LOOP_RATE_ROLL]);
        PID_ResetIntegral(&s_pid[LOOP_RATE_PITCH]);
        PID_ResetIntegral(&s_pid[LOOP_RATE_YAW]);
    }

    Mixer_Compute(cmd->throttle_us, roll_out, pitch_out, yaw_out,
                  ESC_MIN_US, ESC_MAX_US, motors);

    dbg->roll_rate_sp  = roll_rate_sp;
    dbg->pitch_rate_sp = pitch_rate_sp;
}

static void trim_capture_step(const IMUData *imu)
{
    if (fabsf(imu->gyro_x) > TRIM_MAX_RATE_DPS ||
        fabsf(imu->gyro_y) > TRIM_MAX_RATE_DPS ||
        fabsf(imu->gyro_z) > TRIM_MAX_RATE_DPS) {
        s_trim_active = 0;
        Log_Print("[TRIM] aborted — board moved, keep it still\r\n");
        return;
    }

    s_trim_sum_roll  += imu->roll;
    s_trim_sum_pitch += imu->pitch;
    if (++s_trim_count < TRIM_SAMPLES) {
        return;
    }

    s_trim_active = 0;
    s_trim_roll   = s_trim_sum_roll  / (float)TRIM_SAMPLES;
    s_trim_pitch  = s_trim_sum_pitch / (float)TRIM_SAMPLES;
    Log_Printf("[TRIM] roll=%.2f pitch=%.2f (RAM only; copy to flight_cfg.h)\r\n",
               s_trim_roll, s_trim_pitch);
}

void FlightCtrl_Idle(const IMUData *imu)
{
    reset_all();

    if (s_trim_active && imu != NULL) {
        trim_capture_step(imu);
    }
}

void FlightCtrl_SetMode(FlightMode_t mode)
{
    s_mode = mode;
    reset_all();
}

FlightMode_t FlightCtrl_GetMode(void)
{
    return s_mode;
}

void FlightCtrl_SetGains(FlightLoop_t loop, float kp, float ki, float kd)
{
    if (loop >= LOOP_COUNT) return;
    PID_SetGains(&s_pid[loop], kp, ki, kd);
}

const PID_t *FlightCtrl_GetPid(FlightLoop_t loop)
{
    return (loop < LOOP_COUNT) ? &s_pid[loop] : NULL;
}

void FlightCtrl_StartTrimCapture(void)
{
    s_trim_count     = 0;
    s_trim_sum_roll  = 0.0f;
    s_trim_sum_pitch = 0.0f;
    s_trim_active    = 1;
    Log_Print("[TRIM] capturing 2 s — keep the frame level and still\r\n");
}

void FlightCtrl_GetTrim(float *roll_deg, float *pitch_deg)
{
    *roll_deg  = s_trim_roll;
    *pitch_deg = s_trim_pitch;
}
