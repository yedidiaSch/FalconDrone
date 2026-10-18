#ifndef __FLIGHT_CTRL_H__
#define __FLIGHT_CTRL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "imu_service.h"
#include "flight_input.h"
#include "pid.h"

/*
 * Cascaded attitude controller — owns every PID, the flight mode and the
 * level trim. ControlTask calls Update() while ARMED and Idle() otherwise.
 *
 *   ANGLE mode:  angle sp ─▶ [angle P] ─▶ rate sp ─▶ [rate PID] ─▶ mixer
 *   RATE  mode:               rate sp = 0 ─────────▶ [rate PID] ─▶ mixer
 *
 * Yaw is always a rate loop (setpoint FlightCmd.yaw_dps).
 *
 * The rate PIDs integrate only while throttle > LIFTOFF_US, so a drone
 * sitting tilted on the ground cannot wind up an I-term and flip on lift-off.
 *
 * All functions run in ControlTask context (UART commands are parsed there
 * too), so no locking is needed.
 */

typedef enum {
    FLIGHT_MODE_RATE  = 0,
    FLIGHT_MODE_ANGLE = 1,
} FlightMode_t;

typedef enum {
    LOOP_RATE_ROLL = 0,
    LOOP_RATE_PITCH,
    LOOP_RATE_YAW,
    LOOP_ANGLE_ROLL,
    LOOP_ANGLE_PITCH,
    LOOP_COUNT
} FlightLoop_t;

/* Intermediate signals, for telemetry. */
typedef struct {
    float roll_rate_sp;    /* deg/s, output of the angle loop (0 in RATE mode) */
    float pitch_rate_sp;
} FlightCtrlDebug;

void FlightCtrl_Init(void);

/* One control step while ARMED. Writes the four motor pulses. */
void FlightCtrl_Update(const IMUData *imu, const FlightCmd *cmd, float dt,
                       uint16_t motors[4], FlightCtrlDebug *dbg);

/* Every tick while not ARMED: clears PID state and runs a pending trim
 * capture. `imu` may be NULL (no sample this tick). */
void FlightCtrl_Idle(const IMUData *imu);

void          FlightCtrl_SetMode(FlightMode_t mode);
FlightMode_t  FlightCtrl_GetMode(void);

void          FlightCtrl_SetGains(FlightLoop_t loop, float kp, float ki, float kd);
const PID_t  *FlightCtrl_GetPid(FlightLoop_t loop);

/* Average the filter angle for 2 s (board must be still) and use it as the
 * level trim. Result is logged so it can be copied into flight_cfg.h. */
void FlightCtrl_StartTrimCapture(void);
void FlightCtrl_GetTrim(float *roll_deg, float *pitch_deg);

#ifdef __cplusplus
}
#endif

#endif /* __FLIGHT_CTRL_H__ */
