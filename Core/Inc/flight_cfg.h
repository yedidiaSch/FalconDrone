#ifndef __FLIGHT_CFG_H__
#define __FLIGHT_CFG_H__

/*
 * Flight-controller tuning constants — one place for everything that gets
 * adjusted on the bench. Gains here are boot defaults; they can be changed
 * live over UART ($g ...) and the final tuned values copied back here.
 */

/* ---- Throttle ------------------------------------------------------------ */
#define THROTTLE_MAX_US        1500.0f  /* tuning cap — raise only after M4 (tethered hover) */
#define THROTTLE_SLEW_US_S      500.0f  /* max throttle change per second (a jump to t1500 takes ~1 s) */
#define LIFTOFF_US             1150.0f  /* below this the drone is on the ground: I-terms held at 0 */

/* ---- Angle loop (outer, self-level) -------------------------------------- */
#define ANGLE_KP                  2.0f  /* (deg/s) per deg — start low, see design doc §6 phase B */
#define ANGLE_MAX_RATE_DPS      150.0f  /* clamp on the rate setpoint the angle loop may request */
#define ANGLE_MAX_DEG            15.0f  /* stick limit once RC input exists */

/* Level trim: filter angle when the FRAME sits level. Measure with $L on a
 * known-flat surface and copy the logged values here. */
#define TRIM_ROLL_DEG             0.0f
#define TRIM_PITCH_DEG            0.0f

/* ---- Rate loop (inner) — units: us of motor command per deg/s ------------- */
#define RATE_RP_KP                0.6f
#define RATE_RP_KI                0.0f
#define RATE_RP_KD                0.02f
#define RATE_YAW_KP               1.5f
#define RATE_YAW_KI               0.0f
#define RATE_I_LIMIT            200.0f  /* |integral| clamp, deg/s * s */
#define RATE_OUT_LIMIT          300.0f  /* per-axis authority, us */

/* Boot mode. RATE while tuning the inner loop (design doc M2); switch to
 * ANGLE with $m1, and make it the default once the angle loop is tuned (M3). */
#define FLIGHT_MODE_DEFAULT     FLIGHT_MODE_RATE

#endif /* __FLIGHT_CFG_H__ */
