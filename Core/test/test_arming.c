/*
 * Host tests for the arm / disarm state machine (arming.c), with the IMU
 * calibration and the logger stubbed out.
 *
 *   make -C Core/test
 */
#include "arming.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static int s_fail, s_checks;

#define CHECK(cond) do { s_checks++; if (!(cond)) { \
    s_fail++; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)

/* ---- Stubs ---------------------------------------------------------------- */

static ImuCalState s_cal = IMU_CAL_IDLE;
static int  s_cal_requests;
static char s_last_log[160];

void imuRequestCalibration(void)  { s_cal_requests++; s_cal = IMU_CAL_RUNNING; }
ImuCalState imuGetCalState(void)  { return s_cal; }

bool Log_Print(const char *msg)
{
    snprintf(s_last_log, sizeof(s_last_log), "%s", msg);
    return true;
}

bool Log_Printf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_last_log, sizeof(s_last_log), fmt, ap);
    va_end(ap);
    return true;
}

/* ---- Helpers -------------------------------------------------------------- */

static const IMUData LEVEL = { 0 };
static uint32_t s_now = 1000;

/* One 10 ms control tick. */
static ArmState_t tick(uint8_t has_switch, uint8_t sw, uint8_t throttle_low, uint8_t b1)
{
    const ArmInputs in = { .button = b1, .has_switch = has_switch,
                           .arm_switch = sw, .throttle_low = throttle_low };
    s_now += 10;
    /* While calibrating, the IMU task is blocked: no sample. */
    const IMUData *imu = (s_cal == IMU_CAL_RUNNING) ? NULL : &LEVEL;
    return Arming_Update(s_now, &in, imu);
}

static ArmState_t ticks(int n, uint8_t has_switch, uint8_t sw, uint8_t throttle_low)
{
    ArmState_t st = ARM_STATE_DISARMED;
    for (int i = 0; i < n; i++) st = tick(has_switch, sw, throttle_low, 0);
    return st;
}

/* Back to DISARMED, no controller, switch memory cleared. */
static void reset(void)
{
    Arming_Disarm("test reset");
    s_cal = IMU_CAL_IDLE;
    tick(0, 0, 1, 0);
}

/* Calibration finishes, then the 0.5 s level-and-still window. */
static ArmState_t finish_arming(uint8_t sw)
{
    s_cal = IMU_CAL_OK;
    return ticks((int)ARM_CHECK_MS / 10 + 2, 1, sw, 1);
}

/* ---- Tests ---------------------------------------------------------------- */

static void test_switch_flip_arms_and_off_disarms(void)
{
    reset();
    CHECK(tick(1, 0, 1, 0) == ARM_STATE_DISARMED);
    const int req = s_cal_requests;
    CHECK(tick(1, 1, 1, 0) == ARM_STATE_ARMING);           /* OFF→ON */
    CHECK(s_cal_requests == req + 1);
    CHECK(finish_arming(1) == ARM_STATE_ARMED);
    CHECK(tick(1, 1, 1, 0) == ARM_STATE_ARMED);            /* stays */
    CHECK(tick(1, 0, 1, 0) == ARM_STATE_DISARMED);         /* OFF: at once */
    CHECK(strstr(s_last_log, "arm switch off") != NULL);
}

static void test_switch_on_at_connect_does_not_arm(void)
{
    reset();
    CHECK(ticks(50, 1, 1, 1) == ARM_STATE_DISARMED);       /* ON from the start */
    CHECK(tick(1, 0, 1, 0) == ARM_STATE_DISARMED);
    CHECK(tick(1, 1, 1, 0) == ARM_STATE_ARMING);           /* now a real flip */
}

static void test_throttle_up_refuses_until_reflipped(void)
{
    reset();
    tick(1, 0, 0, 0);
    CHECK(tick(1, 1, 0, 0) == ARM_STATE_DISARMED);         /* flip, throttle up */
    CHECK(strstr(s_last_log, "throttle not at zero") != NULL);
    CHECK(ticks(20, 1, 1, 1) == ARM_STATE_DISARMED);       /* throttle down, still ON */
    tick(1, 0, 1, 0);
    CHECK(tick(1, 1, 1, 0) == ARM_STATE_ARMING);
}

static void test_switch_off_aborts_arming(void)
{
    reset();
    tick(1, 0, 1, 0);
    CHECK(tick(1, 1, 1, 0) == ARM_STATE_ARMING);
    CHECK(tick(1, 0, 1, 0) == ARM_STATE_DISARMED);
}

static void test_failed_calibration_needs_reflip(void)
{
    reset();
    tick(1, 0, 1, 0);
    CHECK(tick(1, 1, 1, 0) == ARM_STATE_ARMING);
    s_cal = IMU_CAL_FAILED;
    CHECK(tick(1, 1, 1, 0) == ARM_STATE_DISARMED);
    s_cal = IMU_CAL_IDLE;
    CHECK(ticks(20, 1, 1, 1) == ARM_STATE_DISARMED);       /* no retry by itself */
    tick(1, 0, 1, 0);
    CHECK(tick(1, 1, 1, 0) == ARM_STATE_ARMING);
}

static void test_link_back_with_switch_on_does_not_arm(void)
{
    reset();
    tick(1, 0, 1, 0);
    tick(1, 1, 1, 0);
    CHECK(finish_arming(1) == ARM_STATE_ARMED);
    Arming_Disarm("radio link lost");                     /* what ControlTask does */
    tick(0, 0, 1, 0);                                      /* link down */
    CHECK(ticks(20, 1, 1, 1) == ARM_STATE_DISARMED);       /* back, switch still ON */
}

static void test_b1_without_controller_still_arms(void)
{
    reset();
    CHECK(tick(0, 0, 1, 1) == ARM_STATE_DISARMED);
    ArmState_t st = ARM_STATE_DISARMED;
    for (int i = 0; i < (int)ARM_HOLD_MS / 10 + 1; i++) st = tick(0, 0, 1, 1);
    CHECK(st == ARM_STATE_ARMING);
    s_cal = IMU_CAL_OK;
    CHECK(ticks((int)ARM_CHECK_MS / 10 + 2, 0, 0, 1) == ARM_STATE_ARMED);
    CHECK(tick(0, 0, 1, 1) == ARM_STATE_DISARMED);         /* B1 press disarms */
}

static void test_switch_off_overrides_b1(void)
{
    reset();
    ArmState_t st = ARM_STATE_DISARMED;
    for (int i = 0; i < (int)ARM_HOLD_MS / 10 + 5; i++) st = tick(1, 0, 1, 1);
    CHECK(st == ARM_STATE_DISARMED);                       /* controller says OFF */
}

static void test_pc_disarm_request(void)
{
    reset();
    tick(1, 0, 1, 0);
    tick(1, 1, 1, 0);
    CHECK(finish_arming(1) == ARM_STATE_ARMED);
    Arming_RequestDisarm();
    CHECK(tick(1, 1, 1, 0) == ARM_STATE_DISARMED);
    CHECK(ticks(10, 1, 1, 1) == ARM_STATE_DISARMED);       /* no re-arm while ON */
}

int main(void)
{
    test_switch_flip_arms_and_off_disarms();
    test_switch_on_at_connect_does_not_arm();
    test_throttle_up_refuses_until_reflipped();
    test_switch_off_aborts_arming();
    test_failed_calibration_needs_reflip();
    test_link_back_with_switch_on_does_not_arm();
    test_b1_without_controller_still_arms();
    test_switch_off_overrides_b1();
    test_pc_disarm_request();

    printf("%s: %d checks, %d failed\n", s_fail ? "FAIL" : "OK", s_checks, s_fail);
    return s_fail ? 1 : 0;
}
