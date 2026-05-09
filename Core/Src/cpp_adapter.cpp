/**
 * @file cpp_adapter.cpp
 * @brief Bridge between C system calls and C++ Application Logic.
 * 
 * This file contains the implementation of the global wrapper functions
 * that manage the singleton instance of the MPU6050 driver.
 * 
 * @author Yedidya Schwartz
 */

#include "cpp_adapter.h"
#include "MPU6050.h"
#include <cstdio>

extern "C" {
#include "logger.h"
#include "FreeRTOS.h"
#include "task.h"
}

/* Link to the peripherals and semaphores from C code */
extern "C" I2C_HandleTypeDef hi2c1;
extern "C" osSemaphoreId_t imuSemHandle;
extern "C" osSemaphoreId_t imuDataReadySemHandle;

/* Set by imuRequestCalibration(), consumed by imuTick(). One byte = atomic
 * load/store on Cortex-M4 so a plain volatile is sufficient. */
static volatile uint8_t s_recal_request = 0;

/* Single static instance with TWO arguments */
static MPU6050 imu(&hi2c1, imuSemHandle);


/**
 * @brief Wrapper for initializing the MPU6050 sensor.
 */
void imuInit()
{
    printf("\r\n--- Initializing IMU Object ---\r\n");  // Blocking OK at startup
    if (imu.Init()) 
    {
        printf("IMU Initialization Success!\r\n");
    } 
    else 
    {
        printf("IMU Initialization FAILED!\r\n");
    }
}

/**
 * @brief Wrapper for the threaded IMU update loop.
 * 
 * Executes the read-wait-process cycle for the sensor.
 */
#define IMU_TIMEOUT_MS 200

void imuTick()
{
    /* Service a pending calibration request before kicking off the next read.
     * I2C is idle here (no DMA in flight), so blocking reads in Calibrate()
     * are safe. ControlTask will timeout-fallback during the ~1 s window. */
    if (s_recal_request) {
        s_recal_request = 0;
        Log_Print("[IMU] Calibrating gyro — hold still 1s\r\n");
        imu.Calibrate();
        Log_Printf("[IMU] Bias: x=%+.2f y=%+.2f z=%+.2f dps\r\n",
                   imu.gyro_bias_x, imu.gyro_bias_y, imu.gyro_bias_z);
        return;
    }

    // 1. Trigger the hardware to start reading
    if (!imu.StartUpdateDMA()) {
        // DMA failed to start, skip this cycle
        return;
    }

    // 2. Wait for the semaphore with timeout (prevents deadlock)
    osStatus_t status = osSemaphoreAcquire(imuSemHandle, IMU_TIMEOUT_MS);
    
    if (status == osOK) {
        // 3. Hardware finished! Process the data
        imu.ProcessData();

        // Wake the control task — fresh sample ready.
        // Telemetry is logged from ControlTask so every record has matching
        // IMU + motor values. Don't double-log here.
        osSemaphoreRelease(imuDataReadySemHandle);
    } else {
        // Timeout: I2C may be stuck, will recover on next StartUpdateDMA()
        Log_Printf("IMU Timeout!\r\n");
    }
}

extern "C" void imuRequestCalibration(void)
{
    s_recal_request = 1;
}

extern "C" void imuGetSnapshot(IMUData *out)
{
    taskENTER_CRITICAL();
    out->roll   = imu.roll;
    out->pitch  = imu.pitch;
    out->gyro_x = imu.gyro_x;
    out->gyro_y = imu.gyro_y;
    out->gyro_z = imu.gyro_z;
    taskEXIT_CRITICAL();
}

