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
 *   $t1350            throttle target, us             (ARMED only)
 *   $t+10 / $t-10     nudge throttle target           (ARMED only)
 *   $m0 / $m1         RATE / ANGLE mode               (DISARMED only)
 *   $g rr 0.6 0 0.02  set kp ki kd of a loop: rr rp ry (rate roll/pitch/yaw),
 *                     ar ap (angle roll/pitch)        (any time)
 *   $L                capture level trim, 2 s still   (DISARMED only)
 *   $?                print mode, gains, trim
 *
 * Arming stays physical (B1 hold) — there is no arm command.
 */

/* Feed one received byte. Called from HAL_UART_RxCpltCallback (ISR). */
void UartCmd_OnRxByte(uint8_t byte);

/* Parse and execute a pending line command, if any. ControlTask only. */
void UartCmd_Poll(ArmState_t arm);

#ifdef __cplusplus
}
#endif

#endif /* __UART_CMD_H__ */
