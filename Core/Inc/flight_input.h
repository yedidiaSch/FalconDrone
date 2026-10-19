#ifndef __FLIGHT_INPUT_H__
#define __FLIGHT_INPUT_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Pilot command source for the control loop.
 *
 * Two sources feed it (ControlTask decides which):
 *   - radio link up: throttle and sticks from the controller, every tick
 *   - no radio:      throttle from the PC over UART ($t commands), sticks 0
 *
 * Sticks are normalized (-1..+1). FlightCtrl turns them into setpoints for
 * the current mode: degrees in ANGLE mode, deg/s in RATE mode.
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
    float roll;          /* stick, -1..+1, + = roll right             */
    float pitch;         /* stick, -1..+1, + = nose up                */
    float yaw;           /* stick, -1..+1, + = yaw right (nose right) */
} FlightCmd;

void FlightInput_Reset(void);

/* Absolute target, us. */
void FlightInput_SetThrottle(float target_us);

/* Relative nudge, us (e.g. +10 / -10 from the arrow keys). */
void FlightInput_NudgeThrottle(float delta_us);

/* Stick positions, -1..+1 each (clamped). FlightInput_Reset() centers them. */
void FlightInput_SetSticks(float roll, float pitch, float yaw);

/* Advance the throttle ramp by dt seconds and return the current command. */
FlightCmd FlightInput_Get(float dt);

#ifdef __cplusplus
}
#endif

#endif /* __FLIGHT_INPUT_H__ */
