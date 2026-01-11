/**
 * @file MPU6050.h
 * @brief Driver class for MPU6050 IMU.
 * 
 * Provides an interface for reading Accelerometer and Gyroscope data
 * from the MPU6050 sensor using I2C with DMA and Semaphore synchronization.
 * 
 * @author Yedidya Schwartz
 * @date 2026
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "i2c.h"
#include "cmsis_os2.h" // For Semaphore support
#include <cmath>

/**
 * @class MPU6050
 * @brief Handles MPU6050 sensor operations.
 */
class MPU6050 {
public:
    /**
     * @brief Constructor for MPU6050 driver.
     * 
     * @param hi2c Pointer to the HAL I2C handle used for communication.
     * @param sem Semaphore handle used for synchronizing DMA completion.
     */
    MPU6050(I2C_HandleTypeDef* hi2c, osSemaphoreId_t sem);

    /**
     * @brief Initializes the sensor.
     * 
     * Wakes up the sensor from sleep mode by writing to the Power Management register.
     * 
     * @return true if initialization command was successful, false otherwise.
     */
    bool Init();
    
    /**
     * @brief Triggers a non-blocking DMA read sequence.
     * 
     * Initiates an I2C DMA read of 14 bytes starting from the ACCEL_XOUT_H register.
     */
    void StartUpdateDMA();

    /**
     * @brief Processes the raw buffer after DMA completion.
     * 
     * Converts raw byte data into floating point physical units:
     * - Accelerometer: Gs
     * - Gyroscope: Degrees per second
     * - Roll/Pitch: Degrees
     */
    void ProcessData();

    float roll;     /**< Calculated Roll angle in degrees */
    float pitch;    /**< Calculated Pitch angle in degrees */
    float gyro_x;   /**< Angular velocity X in dps */
    float gyro_y;   /**< Angular velocity Y in dps */
    float gyro_z;   /**< Angular velocity Z in dps */

private:
    I2C_HandleTypeDef* _hi2c;   /**< Handle to I2C peripheral */
    osSemaphoreId_t _sem;       /**< Handle to completion semaphore */
    uint8_t _buffer[14];        /**< Raw data buffer (Accel, Temp, Gyro) */
    
    static const uint16_t DEVICE_ADDR = (0x68 << 1); /**< MPU6050 I2C Address (shifted) */
    static constexpr float RAD_TO_DEG = 180.0f / M_PI;

    // Sensitivities for MPU6050 (Default settings)
    static constexpr float ACCEL_SENSITIVITY = 16384.0f; /**< LSB per G (Range +/- 2g) */
    static constexpr float GYRO_SENSITIVITY  = 131.0f;   /**< LSB per dps (Range +/- 250dps) */
    static constexpr float MPU_BLOCK_SIZE    = 14;       /**< 6 Accel + 2 Temp + 6 Gyro */
};

#endif // MPU6050_H_