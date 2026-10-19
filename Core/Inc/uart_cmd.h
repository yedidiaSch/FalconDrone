#ifndef __UART_CMD_H__
#define __UART_CMD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "arming.h"

/*
 * PC command channel on USART2 (telemetry.py).
 *
 * Single-byte commands act immediately, in the RX interrupt:
 *   'd' or ' '   disarm
 *   'h'          watchdog bench test (Debug builds, DISARMED only)
 *
 * Line commands start with '$' and end with '\n'. Inside a line, 'd' and
 * ' ' are ordinary characters. Lines are parsed in ControlTask context by
 * UartCmd_Poll():
 *
 *   $t1350            throttle target, us             (ARMED, no radio link)
 *   $t+10 / $t-10     nudge throttle target           (ARMED, no radio link)
 *   $m0 / $m1         RATE / ANGLE mode               (DISARMED only)
 *   $g rr 0.6 0 0.02  set kp ki kd of a loop: rr rp ry (rate roll/pitch/yaw),
 *                     ar ap (angle roll/pitch)        (any time)
 *   $L                capture level trim, 2 s still   (DISARMED only)
 *   $?                print mode, gains, trim, radio link
 *   $s r 60 300 3     step test: axis r|p|y, amp (deg/s, or deg in ANGLE
 *                     mode), step ms (default 300 / 800 in ANGLE), reps
 *                     (default 3) — ARMED, props on a rig (tuning.h)
 *   $x                stop the step test
 *   $l1 r / $l0       fast tuning log on (axis r|p|y) / off
 *
 * Arming stays physical (B1 or controller button hold) — there is no arm
 * command.
 */

/* Feed one received byte. Called from HAL_UART_RxCpltCallback (ISR). */
void UartCmd_OnRxByte(uint8_t byte);

/* Parse and execute a pending line command, if any. ControlTask only. */
void UartCmd_Poll(ArmState_t arm, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* __UART_CMD_H__ */
