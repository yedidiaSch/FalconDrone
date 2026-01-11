/**
 * @file MPU6050.cpp
 * @brief Implementation of DMA-based data acquisition.
 */

#include "MPU6050.h"

MPU6050::MPU6050(I2C_HandleTypeDef* hi2c, osSemaphoreId_t sem) 
    : _hi2c(hi2c), _sem(sem), roll(0.0f), pitch(0.0f) 
{
}

bool MPU6050::Init() {
    uint8_t wake_up_data = 0;
    // Wake up using standard blocking call (only happens once at start)
    return (HAL_I2C_Mem_Write(_hi2c, DEVICE_ADDR, 0x6B, 1, &wake_up_data, 1, 100) == HAL_OK);
}

void MPU6050::StartUpdateDMA() {
    // Start non-blocking DMA read
    HAL_I2C_Mem_Read_DMA(_hi2c, DEVICE_ADDR, 0x3B, 1, _buffer, MPU_BLOCK_SIZE);
}

void MPU6050::ProcessData() {
    
    // --- Accelerometer Raw Data ---
    int16_t raw_ax = (int16_t)(_buffer[0] << 8 | _buffer[1]);
    int16_t raw_ay = (int16_t)(_buffer[2] << 8 | _buffer[3]);
    int16_t raw_az = (int16_t)(_buffer[4] << 8 | _buffer[5]);

    // --- Gyroscope Raw Data (Starting from index 8, skipping Temperature) ---
    int16_t raw_gx = (int16_t)(_buffer[8] << 8  | _buffer[9]);
    int16_t raw_gy = (int16_t)(_buffer[10] << 8 | _buffer[11]);
    int16_t raw_gz = (int16_t)(_buffer[12] << 8 | _buffer[13]);

    // Convert to Physical Units
    // Accel in Gs
    float ax = raw_ax / ACCEL_SENSITIVITY;
    float ay = raw_ay / ACCEL_SENSITIVITY;
    float az = raw_az / ACCEL_SENSITIVITY;

    // Gyro in Degrees Per Second (DPS)
    gyro_x = raw_gx / GYRO_SENSITIVITY;
    gyro_y = raw_gy / GYRO_SENSITIVITY;
    gyro_z = raw_gz / GYRO_SENSITIVITY;

    // Static Angle Calculation (Accelerometer only)
    roll  = atan2(ay, az) * RAD_TO_DEG;
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
}