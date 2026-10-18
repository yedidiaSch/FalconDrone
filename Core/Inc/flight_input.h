#ifndef __FLIGHT_INPUT_H__
#define __FLIGHT_INPUT_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Pilot command source for the control loop.
 *
 * Today the throttle comes from the PC over UART ($t commands, tether tests
 * only) and the angle / yaw setpoints are fixed at zero. The RC receiver will
 * later fill the same FlightCmd, so ControlTask does not change.
 *
 * Throttle rules:
 *   - the target is only accepted while ARMED and is clamped to
 *     [ESC_MIN_US, THROTTLE_MAX_US]
 *   - the commanded value ramps toward the target at THROTTLE_SLEW_US_S
 *   - FlightInput_Reset() (called whenever not ARMED) drops both to
 *     ESC_MIN_US, so every arm starts at idle
 *
 * All functions are called from ControlTask only — no locking.
 */

typedef struct {
    float throttle_us;   /* collective, ESC_MIN_US .. THROTTLE_MAX_US */
    float roll_deg;      /* angle setpoint (angle mode)               */
    float pitch_deg;
    float yaw_dps;       /* yaw rate setpoint                         */
} FlightCmd;

void FlightInput_Reset(void);

/* Absolute target, us. */
void FlightInput_SetThrottle(float target_us);

/* Relative nudge, us (e.g. +10 / -10 from the arrow keys). */
void FlightInput_NudgeThrottle(float delta_us);

/* Advance the throttle ramp by dt seconds and return the current command. */
FlightCmd FlightInput_Get(float dt);

#ifdef __cplusplus
}
#endif

#endif /* __FLIGHT_INPUT_H__ */
