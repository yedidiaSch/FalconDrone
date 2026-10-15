/**
 * @file imu_service.h
 * @brief IMU service — C API over the C++ MPU6050 driver + attitude filter.
 *
 * Owns everything around the sensor: the IMU task loop (fixed 100 Hz via
 * osDelayUntil), DMA completion signalling, I2C bus recovery, gyro
 * calibration requests, and the hand-off of each sample to the control loop
 * through a single-slot mailbox queue.
 *
 * Usage:
 *   MX_FREERTOS_Init:  imuCreate(&hi2c1);         (before tasks run)
 *   IMUTask:           imuTaskRun();               (never returns)
 *   ControlTask:       imuWaitSample(&s, 100);
 *   HAL I2C callbacks: imuOnI2cRxComplete / imuOnI2cError
 *
 * @author Yedidya Schwartz
 */

#ifndef IMU_SERVICE_H
#define IMU_SERVICE_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** IMU sample period. The filter and the PID both rely on it being fixed. */
#define IMU_PERIOD_MS 10u

/**
 * @brief One fused IMU sample, delivered by value through the mailbox.
 *
 * Roll/pitch are in degrees (complementary-filter fused).
 * Gyro rates are in degrees/second (bias-corrected).
 */
typedef struct {
    uint32_t t_ms;   /**< Scheduled sample time — exact multiple of IMU_PERIOD_MS */
    float roll;
    float pitch;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} IMUData;

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
 * @brief Bind the I2C bus and create the service's RTOS objects.
 *
 * Call once from MX_FREERTOS_Init, before the scheduler starts, so the
 * mailbox exists before any task can wait on it.
 */
void imuCreate(I2C_HandleTypeDef *hi2c);

/**
 * @brief IMU task body: init the sensor, then read / fuse / publish at a
 *        fixed IMU_PERIOD_MS. Never returns.
 */
void imuTaskRun(void);

/**
 * @brief Block until the next sample is published, then copy it to `out`.
 * @return false on timeout (IMU dead, stuck bus, or calibrating).
 */
bool imuWaitSample(IMUData *out, uint32_t timeout_ms);

/**
 * @brief Request a gyro DC-bias recalibration on the next IMU tick.
 *
 * Returns immediately. The calibration blocks the IMU task for ~1 s —
 * no samples are published during that window, so imuWaitSample() times
 * out and ControlTask falls back to ESC_MIN_US. Poll imuGetCalState().
 *
 * ISR-safe. Multiple calls before the IMU task picks it up coalesce.
 */
void imuRequestCalibration(void);

/**
 * @brief Current state of the calibration requested by imuRequestCalibration().
 */
ImuCalState imuGetCalState(void);

/** @brief Forward from HAL_I2C_MemRxCpltCallback (ISR context). */
void imuOnI2cRxComplete(I2C_HandleTypeDef *hi2c);

/** @brief Forward from HAL_I2C_ErrorCallback (ISR context). */
void imuOnI2cError(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif // IMU_SERVICE_H
