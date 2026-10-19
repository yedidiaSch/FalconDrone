#include "arming.h"
#include "logger.h"
#include <math.h>
#include <stddef.h>

typedef enum {
    PHASE_CALIBRATING = 0,
    PHASE_CHECKING,
} ArmingPhase_t;

/* Written only by ControlTask (via Arming_Update); read by DefaultTask for
 * the LED. Single word — atomic on Cortex-M4. */
static volatile ArmState_t s_state = ARM_STATE_DISARMED;

/* Set from the UART RX ISR, consumed by Arming_Update. */
static volatile uint8_t s_disarm_request = 0;

static ArmingPhase_t s_phase;
static uint32_t      s_phase_start_ms;

/* Button edge / hold tracking. `consumed` stops one press from triggering two
 * actions — e.g. the press that disarms must be released before a new
 * 2 s hold can arm again. */
static uint8_t  s_btn_prev;
static uint8_t  s_btn_consumed;
static uint32_t s_btn_press_ms;

static void disarm(const char *reason)
{
    if (s_state != ARM_STATE_DISARMED) {
        Log_Printf("[ARM] DISARMED: %s\r\n", reason);
    }
    s_state = ARM_STATE_DISARMED;
}

static uint8_t is_level_and_still(const IMUData *imu, const char **reason)
{
    if (fabsf(imu->roll) > ARM_MAX_TILT_DEG || fabsf(imu->pitch) > ARM_MAX_TILT_DEG) {
        *reason = "not level";
        return 0;
    }
    if (fabsf(imu->gyro_x) > ARM_MAX_RATE_DPS ||
        fabsf(imu->gyro_y) > ARM_MAX_RATE_DPS ||
        fabsf(imu->gyro_z) > ARM_MAX_RATE_DPS) {
        *reason = "board moving";
        return 0;
    }
    return 1;
}

static void handle_button(uint32_t now_ms, uint8_t pressed, uint8_t throttle_low)
{
    if (pressed && !s_btn_prev) {
        s_btn_press_ms = now_ms;
        s_btn_consumed = 0;
        if (s_state != ARM_STATE_DISARMED) {
            disarm("button pressed");
            s_btn_consumed = 1;
        }
    }

    if (pressed && !s_btn_consumed && s_state == ARM_STATE_DISARMED &&
        (now_ms - s_btn_press_ms) >= ARM_HOLD_MS) {
        s_btn_consumed = 1;
        if (!throttle_low) {
            Log_Print("[ARM] Refused: throttle not at zero\r\n");
        } else {
            s_state = ARM_STATE_ARMING;
            s_phase = PHASE_CALIBRATING;
            s_phase_start_ms = now_ms;
            imuRequestCalibration();
            Log_Print("[ARM] Arming — release the button and keep the board still\r\n");
        }
    }

    s_btn_prev = pressed;
}

ArmState_t Arming_Update(uint32_t now_ms, const ArmInputs *in, const IMUData *imu)
{
    handle_button(now_ms, in->button, in->throttle_low);

    if (s_disarm_request) {
        s_disarm_request = 0;
        disarm("PC command");
    }

    switch (s_state) {
    case ARM_STATE_DISARMED:
        break;

    case ARM_STATE_ARMING:
        if (s_phase == PHASE_CALIBRATING) {
            /* IMU task is blocked in Calibrate(), so imu == NULL is expected here. */
            const ImuCalState cal = imuGetCalState();
            if (cal == IMU_CAL_OK) {
                s_phase = PHASE_CHECKING;
                s_phase_start_ms = now_ms;
            } else if (cal == IMU_CAL_FAILED) {
                disarm("calibration failed — keep the board still");
            } else if ((now_ms - s_phase_start_ms) > ARM_CAL_TIMEOUT_MS) {
                disarm("calibration timeout");
            }
        } else {
            const char *reason = "";
            if (imu == NULL) {
                disarm("IMU timeout");
            } else if (!is_level_and_still(imu, &reason)) {
                disarm(reason);
            } else if ((now_ms - s_phase_start_ms) >= ARM_CHECK_MS) {
                s_state = ARM_STATE_ARMED;
                Log_Print("[ARM] ARMED — press the button or 'd' on the PC to disarm\r\n");
            }
        }
        break;

    case ARM_STATE_ARMED:
        if (imu == NULL) {
            disarm("IMU timeout");
        } else if (fabsf(imu->roll) > ARM_DISARM_TILT_DEG ||
                   fabsf(imu->pitch) > ARM_DISARM_TILT_DEG) {
            disarm("tilt limit");
        }
        break;
    }

    return s_state;
}

ArmState_t Arming_GetState(void)
{
    return s_state;
}

void Arming_RequestDisarm(void)
{
    s_disarm_request = 1;
}

void Arming_Disarm(const char *reason)
{
    disarm(reason);
}
