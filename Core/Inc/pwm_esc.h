#ifndef __PWM_ESC_H__
#define __PWM_ESC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * Motor layout — F450 X-frame (viewed from above):
 *
 *       Front
 *   M3(CCW)  M1(CW)
 *      \      /
 *       [body]
 *      /      \
 *   M2(CW)  M4(CCW)
 *       Rear
 *
 * Hardware mapping:
 *   M1  Front-Right  CW   TIM2_CH1  PA0
 *   M2  Rear-Left    CW   TIM2_CH2  PA1
 *   M3  Front-Left   CCW  TIM3_CH1  PA6
 *   M4  Rear-Right   CCW  TIM3_CH2  PA7
 *
 * PWM: 50 Hz, 1 µs resolution (timer tick = 1 µs, period = 20 000 ticks)
 * ESC signal range: 1000 µs (min/armed) to 2000 µs (full throttle)
 */

#define ESC_MOTOR_1     0u
#define ESC_MOTOR_2     1u
#define ESC_MOTOR_3     2u
#define ESC_MOTOR_4     3u
#define ESC_MOTOR_COUNT 4u

#define ESC_MIN_US      1000u   /* Minimum valid pulse — armed, zero thrust */
#define ESC_MAX_US      2000u   /* Maximum pulse — full throttle            */
#define ESC_OFF_US      0u      /* No pulse — ESC receives no signal        */

#define ESC_ARM_DELAY_MS    4000u   /* Time to hold ESC_MIN_US before ESCs are armed */
#define ESC_UPDATE_PERIOD_MS  20u   /* 50 Hz motor update cadence                    */

/* Start PWM output on all four channels at ESC_MIN_US. Call once before arming. */
void ESC_Init(void);

/* Send ESC_MIN_US on all channels. After ESC_ARM_DELAY_MS the ESCs are armed. */
void ESC_Arm(void);

/* Send ESC_OFF_US on all channels (no pulse — motors coast/stop). */
void ESC_Disarm(void);

/*
 * Set one motor. Clamps throttle_us to [ESC_MIN_US, ESC_MAX_US].
 * Only call after ESC_Arm() has completed.
 */
void ESC_SetMotor(uint8_t motor, uint16_t throttle_us);

/*
 * Set all four motors in one call (raw — no clamping).
 * Intended for the PID output stage where values are already bounded.
 */
void ESC_SetAllMotors(uint16_t m1_us, uint16_t m2_us,
                      uint16_t m3_us, uint16_t m4_us);

#ifdef __cplusplus
}
#endif

#endif /* __PWM_ESC_H__ */
