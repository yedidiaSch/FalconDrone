#include "tuning.h"
#include "tune_seq.h"
#include "flight_cfg.h"
#include "pwm_esc.h"
#include "logger.h"
#include <math.h>

static const char s_axis_char[FLIGHT_AXIS_COUNT] = { 'r', 'p', 'y' };

static TuneSeq      s_seq;
static FlightAxis_t s_test_axis = FLIGHT_AXIS_NONE;
static uint8_t      s_segment;
static uint32_t     s_now_ms;      /* last Tuning_Tick() time, for $x */

static bool         s_log_manual;
static FlightAxis_t s_log_axis = FLIGHT_AXIS_ROLL;
static uint32_t     s_log_tail_until_ms;
static bool         s_log_tail;

static bool angle_ref(FlightAxis_t axis)
{
    return FlightCtrl_GetMode() == FLIGHT_MODE_ANGLE && axis != FLIGHT_AXIS_YAW;
}

static void finish(const char *reason, uint32_t now_ms)
{
    if (s_test_axis == FLIGHT_AXIS_NONE) {
        return;
    }
    TuneSeq_Stop(&s_seq);
    FlightCtrl_SetTestInput(FLIGHT_AXIS_NONE, 0.0f);
    s_test_axis = FLIGHT_AXIS_NONE;
    s_segment = 0;
    s_log_tail = true;
    s_log_tail_until_ms = now_ms + TUNE_LOG_TAIL_MS;
    if (reason == NULL) {
        Log_Print("[TUNE] done\r\n");
    } else {
        Log_Printf("[TUNE] aborted: %s\r\n", reason);
    }
}

bool Tuning_Start(FlightAxis_t axis, float amp, uint16_t step_ms, uint8_t reps,
                  ArmState_t arm, uint32_t now_ms)
{
    if (axis >= FLIGHT_AXIS_COUNT) {
        Log_Print("[TUNE] usage: $s <r|p|y> <amp> [step_ms] [reps]\r\n");
        return false;
    }
    if (arm != ARM_STATE_ARMED) {
        Log_Print("[TUNE] rejected: arm first (props on, drone on the rig)\r\n");
        return false;
    }
    if (s_test_axis != FLIGHT_AXIS_NONE) {
        Log_Print("[TUNE] rejected: a test is running ($x to stop it)\r\n");
        return false;
    }

    const bool  is_angle = angle_ref(axis);
    const float max_amp  = is_angle ? TUNE_MAX_ANGLE_DEG : TUNE_MAX_RATE_DPS;
    if (fabsf(amp) < 1.0f || fabsf(amp) > max_amp ||
        step_ms < TUNE_MIN_STEP_MS || step_ms > TUNE_MAX_STEP_MS ||
        reps < 1u || reps > TUNE_MAX_REPS) {
        Log_Printf("[TUNE] rejected: amp 1..%.0f %s, step %u..%u ms, reps 1..%u\r\n",
                   max_amp, is_angle ? "deg" : "deg/s",
                   TUNE_MIN_STEP_MS, TUNE_MAX_STEP_MS, TUNE_MAX_REPS);
        return false;
    }

    const TuneSeqCfg cfg = { .amp = amp, .step_ms = step_ms, .reps = reps };
    TuneSeq_Start(&s_seq, &cfg, now_ms);
    s_test_axis = axis;
    s_log_axis  = axis;
    s_log_tail  = false;

    const PID_t *rate  = FlightCtrl_GetPid((FlightLoop_t)(LOOP_RATE_ROLL + axis));
    const PID_t *outer = (axis != FLIGHT_AXIS_YAW)
                       ? FlightCtrl_GetPid((FlightLoop_t)(LOOP_ANGLE_ROLL + axis)) : NULL;
    Log_Printf("[TUNE] start axis=%c mode=%s amp=%.1f step=%u reps=%u "
               "kp=%.3f ki=%.3f kd=%.4f lim=%.0f akp=%.3f\r\n",
               s_axis_char[axis], is_angle ? "ANGLE" : "RATE", amp,
               (unsigned)step_ms, (unsigned)reps,
               rate->kp, rate->ki, rate->kd, rate->out_limit,
               outer != NULL ? outer->kp : 0.0f);
    Log_Printf("[TUNE] runs %lu ms\r\n", (unsigned long)TuneSeq_DurationMs(&cfg));
    return true;
}

void Tuning_Abort(const char *reason)
{
    if (s_test_axis == FLIGHT_AXIS_NONE) {
        Log_Print("[TUNE] no test running\r\n");
        return;
    }
    finish(reason, s_now_ms);
}

void Tuning_SetLog(bool on, FlightAxis_t axis)
{
    s_log_manual = on;
    if (axis < FLIGHT_AXIS_COUNT) {
        s_log_axis = axis;
    }
    if (on) {
        Log_Printf("[TUNE] log on, axis=%c, 100 Hz — T,t_ms,seg,ref,sp,meas,ang,p,i,d,u,thr,sat\r\n",
                   s_axis_char[s_log_axis]);
    } else {
        Log_Print("[TUNE] log off\r\n");
    }
}

static float stick_on(const FlightCmd *cmd, FlightAxis_t axis)
{
    switch (axis) {
    case FLIGHT_AXIS_ROLL:  return cmd->roll;
    case FLIGHT_AXIS_PITCH: return cmd->pitch;
    default:                return cmd->yaw;
    }
}

void Tuning_Tick(uint32_t now_ms, ArmState_t arm, const IMUData *imu, const FlightCmd *cmd)
{
    s_now_ms = now_ms;
    if (s_test_axis == FLIGHT_AXIS_NONE) {
        return;
    }
    if (arm != ARM_STATE_ARMED || cmd == NULL || imu == NULL) {
        finish("disarmed", now_ms);
        return;
    }
    if (fabsf(stick_on(cmd, s_test_axis)) > TUNE_OVERRIDE_STICK) {
        finish("pilot moved the stick", now_ms);
        return;
    }
    if (fabsf(imu->roll) > TUNE_ABORT_TILT_DEG || fabsf(imu->pitch) > TUNE_ABORT_TILT_DEG) {
        finish("tilt limit", now_ms);
        return;
    }

    float ref;
    if (!TuneSeq_Step(&s_seq, now_ms, &ref, &s_segment)) {
        finish(NULL, now_ms);
        return;
    }
    FlightCtrl_SetTestInput(s_test_axis, ref);
}

bool Tuning_Log(uint32_t now_ms, const IMUData *imu, const FlightCmd *cmd,
                const FlightCtrlDebug *dbg, const uint16_t motors[4])
{
    if (s_log_tail && (int32_t)(now_ms - s_log_tail_until_ms) >= 0) {
        s_log_tail = false;
    }
    if (!s_log_manual && !s_log_tail && s_test_axis == FLIGHT_AXIS_NONE) {
        return false;
    }

    const FlightAxis_t axis = s_log_axis;
    const PID_t *pid = FlightCtrl_GetPid((FlightLoop_t)(LOOP_RATE_ROLL + axis));
    float trim_r, trim_p;
    FlightCtrl_GetTrim(&trim_r, &trim_p);

    float sp, meas, ang, ref;
    switch (axis) {
    case FLIGHT_AXIS_ROLL:
        sp = dbg->roll_rate_sp;  meas = imu->gyro_x; ang = imu->roll - trim_r;
        ref = angle_ref(axis) ? dbg->roll_ref : sp;
        break;
    case FLIGHT_AXIS_PITCH:
        sp = dbg->pitch_rate_sp; meas = imu->gyro_y; ang = imu->pitch - trim_p;
        ref = angle_ref(axis) ? dbg->pitch_ref : sp;
        break;
    default:
        sp = dbg->yaw_rate_sp;   meas = imu->gyro_z; ang = 0.0f;
        ref = sp;
        break;
    }

    uint8_t sat = fabsf(pid->output) >= pid->out_limit * 0.999f;
    for (int i = 0; i < 4; i++) {
        if (motors[i] <= ESC_MIN_US || motors[i] >= ESC_MAX_US) {
            sat = 1;
        }
    }

    Log_Printf("T,%lu,%u,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%u,%u\r\n",
               (unsigned long)now_ms, (unsigned)s_segment,
               ref, sp, meas, ang,
               pid->p_term, pid->i_term, pid->d_term, pid->output,
               (unsigned)cmd->throttle_us, (unsigned)sat);
    return true;
}
