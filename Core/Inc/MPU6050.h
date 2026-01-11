/**
 * @file MPU6050.hpp
 * @brief Driver class for MPU6050 with DMA and FreeRTOS synchronization.
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "i2c.h"
#include "cmsis_os2.h" // For Semaphore support
#include <cmath>

class MPU6050 {
public:
    /**
     * @brief Constructor
     * @param hi2c Pointer to HAL I2C handle
     * @param sem Semaphore handle for DMA synchronization
     */
    MPU6050(I2C_HandleTypeDef* hi2c, osSemaphoreId_t sem);

    bool Init();
    
    /**
     * @brief Triggers a non-blocking DMA read
     */
    void StartUpdateDMA();

    /**
     * @brief Processes the buffer after DMA completion
     */
    void ProcessData();

    float roll;
    float pitch;
    float gyro_x;
    float gyro_y;
    float gyro_z;

private:
    I2C_HandleTypeDef* _hi2c;
    osSemaphoreId_t _sem;
    uint8_t _buffer[14]; // DMA Buffer
    
    static const uint16_t DEVICE_ADDR = (0x68 << 1);
    static constexpr float RAD_TO_DEG = 180.0f / M_PI;

    // Sensitivities for MPU6050 (Default settings)
    static constexpr float ACCEL_SENSITIVITY = 16384.0f;
    static constexpr float GYRO_SENSITIVITY  = 131.0f;
    static constexpr float MPU_BLOCK_SIZE    = 14;   // 6 Accel + 2 Temp + 6 Gyro
};

#endif // MPU6050_H_