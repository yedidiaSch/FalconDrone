/**
 * @file cpp_adapter.h
 * @brief C to C++ Adapter Layer
 * 
 * This file declares the C-compatible functions that allow
 * the C-based `main.c` and `freertos.c` to interface with the
 * C++ MPU6050 driver classes.
 * 
 * @author Yedidya Schwartz
 * @date 2026-01-12
 */

#ifndef CPP_ADAPTER_H
#define CPP_ADAPTER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Snapshot of the latest IMU computation, copied out atomically.
 *
 * Roll/pitch are in degrees (complementary-filter fused).
 * Gyro rates are in degrees/second (raw scaled).
 */
typedef struct {
    float roll;
    float pitch;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} IMUData;

// --- Functions callable from C files ---

/**
 * @brief Initialize the global IMU object.
 */
void imuInit();

/**
 * @brief Periodic IMU process tick — DMA read, wait, fuse, signal.
 *
 * On success, releases imuDataReadySemHandle to wake the control task.
 */
void imuTick();

/**
 * @brief Copy the latest IMU values into `out` under a critical section.
 *
 * Safe to call from any task. Cheap (~5 word copy, interrupts off briefly).
 */
void imuGetSnapshot(IMUData *out);

/**
 * @brief Progress of the most recent gyro calibration request.
 */
typedef enum {
    IMU_CAL_IDLE = 0,   /**< No calibration requested since boot          */
    IMU_CAL_PENDING,    /**< Requested, IMU task hasn't picked it up yet  */
    IMU_CAL_RUNNING,    /**< IMU task is sampling (~1 s, board must be still) */
    IMU_CAL_OK,         /**< Finished, new bias applied                   */
    IMU_CAL_FAILED      /**< Board moved or I2C failed — old bias kept    */
} ImuCalState;

/**
 * @brief Request a gyro DC-bias recalibration on the next IMU tick.
 *
 * Sets the state to IMU_CAL_PENDING; imuTick() consumes it at the top of its
 * next cycle. Returns immediately. The actual calibration blocks the IMU task
 * for ~1 s — during that window, ControlTask hits its sem timeout and falls
 * back to ESC_MIN_US. Poll imuGetCalState() for the result.
 *
 * Callable from any context (task, ISR-safe). Idempotent — multiple calls
 * before the IMU task picks it up coalesce into one calibration.
 */
void imuRequestCalibration(void);

/**
 * @brief Current state of the calibration requested by imuRequestCalibration().
 */
ImuCalState imuGetCalState(void);

#ifdef __cplusplus
}
#endif

#endif // CPP_ADAPTER_H