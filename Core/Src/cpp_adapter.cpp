#include "cpp_adapter.h"
#include "MPU6050.h"
#include <cstdio>

/* Link to the peripherals and semaphores from C code */
extern "C" I2C_HandleTypeDef hi2c1;
extern "C" osSemaphoreId_t imuSemHandle;

/* Single static instance with TWO arguments */
static MPU6050 imu(&hi2c1, imuSemHandle);


void imuInit()
{
    printf("\r\n--- Initializing IMU Object ---\r\n");
    if (imu.Init()) 
    {
        printf("IMU Initialization Success!\r\n");
    } 

    else 
    {
        printf("IMU Initialization FAILED!\r\n");
    }
}

void imuTick()
{
    // 1. Trigger the hardware to start reading
    imu.StartUpdateDMA();

    // 2. Wait for the semaphore (Task sleeps here, NO CPU USAGE!)
    if (osSemaphoreAcquire(imuSemHandle, osWaitForever) == osOK) {
        // 3. Hardware finished! Process the data
        imu.ProcessData();
        
        printf("\rR: %6.2f P: %6.2f | Gx: %6.2f Gy: %6.2f Gz: %6.2f", 
               imu.roll, imu.pitch, imu.gyro_x, imu.gyro_y, imu.gyro_z);
        fflush(stdout);
    }
}

