#include "uart_cmd.h"
#include "main.h"
#include "flight_ctrl.h"
#include "flight_input.h"
#include "logger.h"
#include "rc_service.h"
#include "watchdog.h"
#include <stdlib.h>
#include <string.h>

#define CMD_MAX_LEN 48u

/* ISR-side line assembly. */
static uint8_t s_in_line;
static char    s_rx_buf[CMD_MAX_LEN];
static uint8_t s_rx_len;

/* Hand-off: the ISR fills s_line then sets s_line_ready; ControlTask copies
 * it out and clears the flag. A line arriving before the previous one was
 * consumed is dropped. */
static char             s_line[CMD_MAX_LEN];
static volatile uint8_t s_line_ready;

void UartCmd_OnRxByte(uint8_t byte)
{
    if (!s_in_line) {
        if (byte == '$') {
            s_in_line = 1;
            s_rx_len  = 0;
        } else if (byte == 'd' || byte == ' ') {
            Arming_RequestDisarm();
        }
#ifdef DEBUG
        else if (byte == 'h') {
            Watchdog_RequestTestHang();
        }
#endif
        return;
    }

    if (byte == '\n' || byte == '\r') {
        if (s_rx_len > 0 && !s_line_ready) {
            memcpy(s_line, s_rx_buf, s_rx_len);
            s_line[s_rx_len] = '\0';
            __DMB();   /* line contents visible before the flag */
            s_line_ready = 1;
        }
        s_in_line = 0;
    } else if (s_rx_len < CMD_MAX_LEN - 1u) {
        s_rx_buf[s_rx_len++] = (char)byte;
    } else {
        s_in_line = 0;   /* too long — drop the line */
    }
}

static const char *const s_loop_names[LOOP_COUNT] = { "rr", "rp", "ry", "ar", "ap" };

static const char *link_name(RcLinkState st)
{
    switch (st) {
    case RC_LINK_UP:   return "UP";
    case RC_LINK_LOST: return "LOST";
    default:           return "NONE";
    }
}

static void print_config(void)
{
    float trim_r, trim_p;
    FlightCtrl_GetTrim(&trim_r, &trim_p);
    Log_Printf("[CFG] mode=%s trim r=%.2f p=%.2f\r\n",
               FlightCtrl_GetMode() == FLIGHT_MODE_ANGLE ? "ANGLE" : "RATE",
               trim_r, trim_p);
    for (int i = 0; i < LOOP_COUNT; i++) {
        const PID_t *pid = FlightCtrl_GetPid((FlightLoop_t)i);
        Log_Printf("[CFG] %s kp=%.3f ki=%.3f kd=%.4f\r\n",
                   s_loop_names[i], pid->kp, pid->ki, pid->kd);
    }

    const RcLink *link = RcService_Link();
    Log_Printf("[RC] radio=%s link=%s %u%% thr=%u roll=%d pitch=%d flags=0x%02x\r\n",
               RcService_RadioOk() ? "ok" : "MISSING",
               link_name(link->state), (unsigned)link->link_pct,
               (unsigned)link->last.throttle, (int)link->last.roll,
               (int)link->last.pitch, (unsigned)link->last.flags);
}

static void cmd_throttle(const char *arg, ArmState_t arm)
{
    if (arm != ARM_STATE_ARMED) {
        Log_Print("[CMD] throttle rejected: not armed\r\n");
        return;
    }
    if (RcService_Link()->state == RC_LINK_UP) {
        Log_Print("[CMD] throttle rejected: the controller has the throttle\r\n");
        return;
    }
    char *end;
    const float v = strtof(arg, &end);
    if (end == arg) {
        Log_Print("[CMD] usage: $t1350 | $t+10 | $t-10\r\n");
        return;
    }
    if (arg[0] == '+' || arg[0] == '-') {
        FlightInput_NudgeThrottle(v);
    } else {
        FlightInput_SetThrottle(v);
    }
}

static void cmd_mode(const char *arg, ArmState_t arm)
{
    if (arm != ARM_STATE_DISARMED) {
        Log_Print("[CMD] mode change rejected: disarm first\r\n");
        return;
    }
    if (arg[0] == '0') {
        FlightCtrl_SetMode(FLIGHT_MODE_RATE);
    } else if (arg[0] == '1') {
        FlightCtrl_SetMode(FLIGHT_MODE_ANGLE);
    } else {
        Log_Print("[CMD] usage: $m0 (rate) | $m1 (angle)\r\n");
        return;
    }
    Log_Printf("[CMD] mode %s\r\n", arg[0] == '1' ? "ANGLE" : "RATE");
}

static void cmd_gains(const char *arg)
{
    while (*arg == ' ') arg++;

    int loop = -1;
    for (int i = 0; i < LOOP_COUNT; i++) {
        if (strncmp(arg, s_loop_names[i], 2) == 0) {
            loop = i;
            break;
        }
    }

    float k[3];
    const char *p = arg + 2;
    int n = 0;
    while (loop >= 0 && n < 3) {
        char *end;
        k[n] = strtof(p, &end);
        if (end == p) break;
        p = end;
        n++;
    }

    if (loop < 0 || n != 3 || k[0] < 0.0f || k[1] < 0.0f || k[2] < 0.0f) {
        Log_Print("[CMD] usage: $g <rr|rp|ry|ar|ap> kp ki kd\r\n");
        return;
    }
    FlightCtrl_SetGains((FlightLoop_t)loop, k[0], k[1], k[2]);
    Log_Printf("[CMD] %s kp=%.3f ki=%.3f kd=%.4f\r\n",
               s_loop_names[loop], k[0], k[1], k[2]);
}

void UartCmd_Poll(ArmState_t arm)
{
    if (!s_line_ready) {
        return;
    }
    __DMB();   /* pairs with the barrier in the ISR */
    char line[CMD_MAX_LEN];
    memcpy(line, s_line, sizeof(line));
    s_line_ready = 0;

    switch (line[0]) {
    case 't': cmd_throttle(&line[1], arm); break;
    case 'm': cmd_mode(&line[1], arm);     break;
    case 'g': cmd_gains(&line[1]);         break;
    case 'L':
        if (arm == ARM_STATE_DISARMED) {
            FlightCtrl_StartTrimCapture();
        } else {
            Log_Print("[CMD] trim rejected: disarm first\r\n");
        }
        break;
    case '?': print_config(); break;
    default:
        Log_Printf("[CMD] unknown: %s\r\n", line);
        break;
    }
}
