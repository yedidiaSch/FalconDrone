#ifndef __TUNING_H__
#define __TUNING_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "arming.h"
#include "flight_ctrl.h"
#include "flight_input.h"
#include "imu_service.h"

/*
 * Tuning support: automatic step tests and the fast tuning log.
 *
 * Step test ($s, PC only, ARMED only, PROPS ON ON A RIG):
 *   Runs a tune_seq.h doublet on one axis by replacing that axis's stick
 *   setpoint. RATE mode (and yaw): amp in deg/s. ANGLE mode: amp in deg.
 *   Aborts on: disarm (any cause — 'd' on the PC still works), $x, pilot
 *   stick on the tested axis beyond TUNE_OVERRIDE_STICK, |roll| or |pitch|
 *   beyond TUNE_ABORT_TILT_DEG.
 *
 * Tuning log ($l1 <axis>, or on automatically during a step test and for
 * TUNE_LOG_TAIL_MS after it): replaces the 50 Hz CSV with one line per
 * control tick (100 Hz) for one axis:
 *
 *   T,t_ms,seg,ref,sp,meas,ang,p,i,d,u,thr,sat
 *
 *   seg   step-test segment (0 = lead-in / no test), see tune_seq.h
 *   ref   outer reference: angle setpoint (ANGLE mode, roll/pitch) else
 *         the rate setpoint, deg or deg/s
 *   sp    rate setpoint into the rate loop, deg/s
 *   meas  gyro rate, deg/s
 *   ang   filter angle minus trim, deg (0 for yaw)
 *   p,i,d rate PID terms, us;  u = clamped PID output, us
 *   thr   collective throttle, us
 *   sat   1 if a motor sits at ESC_MIN/MAX or u is at its limit
 *
 * Every test logs a "[TUNE] start ..." line with the gains in use, and
 * "[TUNE] done" or "[TUNE] aborted: <reason>" — tools/tune_analyze.py keys
 * on these.
 *
 * ControlTask only (uart_cmd.c runs there too) — no locking.
 */

#define TUNE_MAX_RATE_DPS     150.0f
#define TUNE_MAX_ANGLE_DEG     20.0f
#define TUNE_MIN_STEP_MS          50u
#define TUNE_MAX_STEP_MS        2000u
#define TUNE_MAX_REPS             10u
#define TUNE_ABORT_TILT_DEG    35.0f
#define TUNE_OVERRIDE_STICK     0.25f   /* normalized stick on the tested axis */
#define TUNE_LOG_TAIL_MS        1000u

/* $s — returns true if the test started (reason logged otherwise). */
bool Tuning_Start(FlightAxis_t axis, float amp, uint16_t step_ms, uint8_t reps,
                  ArmState_t arm, uint32_t now_ms);

/* $x */
void Tuning_Abort(const char *reason);

/* $l — manual tuning log on/off. */
void Tuning_SetLog(bool on, FlightAxis_t axis);

/* Every control tick. cmd = pilot command for this tick (NULL when not
 * ARMED). Drives FlightCtrl_SetTestInput(). */
void Tuning_Tick(uint32_t now_ms, ArmState_t arm, const IMUData *imu, const FlightCmd *cmd);

/* Every control tick after FlightCtrl. Returns true if it wrote the tuning
 * log line — the normal CSV is skipped then. */
bool Tuning_Log(uint32_t now_ms, const IMUData *imu, const FlightCmd *cmd,
                const FlightCtrlDebug *dbg, const uint16_t motors[4]);

#ifdef __cplusplus
}
#endif

#endif /* __TUNING_H__ */
