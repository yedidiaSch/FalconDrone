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
}

/* Link to the peripherals and semaphores from C code */
extern "C" I2C_HandleTypeDef hi2c1;
extern "C" osSemaphoreId_t imuSemHandle;

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
        
        // Non-blocking log - returns immediately!
        Log_Printf("%.2f,%.2f\r\n", imu.roll, imu.pitch);
    } else {
        // Timeout: I2C may be stuck, will recover on next StartUpdateDMA()
        Log_Printf("IMU Timeout!\r\n");
    }
}

