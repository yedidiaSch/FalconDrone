#ifndef __ARMING_H__
#define __ARMING_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "imu_service.h"

/*
 * Arm / disarm state machine — decides whether the control loop may drive
 * the motors. Two ways to arm:
 *
 *   - the controller's ARM switch (radio link UP): an OFF→ON flip arms,
 *     OFF disarms at once. A switch that is already ON when the link comes
 *     up, or when an arm attempt failed, must go OFF and ON again.
 *   - the user button B1 (blue, PC13) held 2 s, for the PC-tethered bench.
 *     While a controller is connected its switch still rules: B1 cannot
 *     keep the drone armed with the switch OFF.
 *
 *   DISARMED ──switch ON / B1 2 s──▶ ARMING ──cal OK + 0.5 s level & still──▶ ARMED
 *       ▲                               │                                        │
 *       └── any failure / switch OFF / B1 press / PC 'd' / radio lost ◀──────────┘
 *
 * ARMING runs a fresh gyro calibration (board must be still for ~1 s), then
 * a pre-arm check window. Any failed check aborts back to DISARMED with the
 * reason logged over UART. Arming is refused while the radio throttle is
 * above zero (throttle_low == 0).
 *
 * While ARMED, any of these disarms immediately:
 *   - the ARM switch reading OFF
 *   - a B1 press (the press edge, not the release)
 *   - a 'd' or ' ' byte received on USART2 (telemetry.py sends it)
 *   - no fresh IMU sample for 100 ms (caller passes imu == NULL)
 *   - |roll| or |pitch| beyond ARM_DISARM_TILT_DEG (crash / tip-over)
 *   - Arming_Disarm() from ControlTask (radio link lost)
 *
 * Boot state is always DISARMED.
 */

typedef enum {
    ARM_STATE_DISARMED = 0,
    ARM_STATE_ARMED    = 1,
    ARM_STATE_ARMING   = 2,   /* calibrating + pre-arm checks */
} ArmState_t;

#define ARM_HOLD_MS            2000u  /* B1 hold time to start arming          */
#define ARM_CAL_TIMEOUT_MS     3000u  /* give up if calibration never finishes  */
#define ARM_CHECK_MS            500u  /* level & still window after calibration */
#define ARM_MAX_TILT_DEG       15.0f  /* pre-arm: |roll|, |pitch| below this    */
#define ARM_MAX_RATE_DPS        5.0f  /* pre-arm: |gyro| on every axis below    */
#define ARM_DISARM_TILT_DEG    60.0f  /* armed: beyond this = crash, disarm     */

typedef struct {
    uint8_t button;        /* 1 while B1 is held                                */
    uint8_t has_switch;    /* 1 = controller connected (radio link UP)          */
    uint8_t arm_switch;    /* its ARM switch is ON (valid with has_switch)      */
    uint8_t throttle_low;  /* 0 = radio throttle above zero: refuse to arm      */
} ArmInputs;

/*
 * Run one step. Call from ControlTask on every loop iteration.
 *   now_ms : kernel tick in ms
 *   in     : button and throttle state
 *   imu    : fresh IMU snapshot, or NULL if the sample wait timed out
 * Returns the state after this step.
 */
ArmState_t Arming_Update(uint32_t now_ms, const ArmInputs *in, const IMUData *imu);

/* Current state. Safe to call from any task. */
ArmState_t Arming_GetState(void);

/* Request a disarm on the next Arming_Update(). ISR-safe (UART RX callback). */
void Arming_RequestDisarm(void);

/* Disarm now, logging `reason`. ControlTask only. */
void Arming_Disarm(const char *reason);

#ifdef __cplusplus
}
#endif

#endif /* __ARMING_H__ */
