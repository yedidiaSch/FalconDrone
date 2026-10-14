#ifndef __ARMING_H__
#define __ARMING_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "cpp_adapter.h"

/*
 * Arm / disarm state machine — decides whether the control loop may drive
 * the motors. Until an RC receiver exists, the user button B1 (blue, PC13)
 * is the arm switch.
 *
 *   DISARMED ──hold B1 2 s──▶ ARMING ──cal OK + 0.5 s level & still──▶ ARMED
 *       ▲                        │                                        │
 *       └──── any failure / B1 press / PC 'd' or space ◀──────────────────┘
 *
 * ARMING runs a fresh gyro calibration (board must be still for ~1 s), then
 * a pre-arm check window. Any failed check aborts back to DISARMED with the
 * reason logged over UART.
 *
 * While ARMED, any of these disarms immediately:
 *   - a B1 press (the press edge, not the release)
 *   - a 'd' or ' ' byte received on USART2 (telemetry.py sends it)
 *   - no fresh IMU sample for 100 ms (caller passes imu == NULL)
 *   - |roll| or |pitch| beyond ARM_DISARM_TILT_DEG (crash / tip-over)
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

/*
 * Run one step. Call from ControlTask on every loop iteration.
 *   now_ms     : kernel tick in ms
 *   b1_pressed : 1 while the button is held down
 *   imu        : fresh IMU snapshot, or NULL if the sample wait timed out
 * Returns the state after this step.
 */
ArmState_t Arming_Update(uint32_t now_ms, uint8_t b1_pressed, const IMUData *imu);

/* Current state. Safe to call from any task. */
ArmState_t Arming_GetState(void);

/* Request a disarm on the next Arming_Update(). ISR-safe (UART RX callback). */
void Arming_RequestDisarm(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARMING_H__ */
