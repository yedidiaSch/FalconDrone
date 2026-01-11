/**
 * @file MPU6050.cpp
 * @brief Implementation of DMA-based data acquisition.
 */

#include "MPU6050.h"

MPU6050::MPU6050(I2C_HandleTypeDef* hi2c, osSemaphoreId_t sem) 
    : _hi2c(hi2c), _sem(sem), roll(0.0f), pitch(0.0f) {}

bool MPU6050::Init() {
    uint8_t wake_up_data = 0;
    // Wake up using standard blocking call (only happens once at start)
    return (HAL_I2C_Mem_Write(_hi2c, DEVICE_ADDR, 0x6B, 1, &wake_up_data, 1, 100) == HAL_OK);
}

void MPU6050::StartUpdateDMA() {
    // Start non-blocking DMA read
    HAL_I2C_Mem_Read_DMA(_hi2c, DEVICE_ADDR, 0x3B, 1, _buffer, 6);
}

void MPU6050::ProcessData() {
    int16_t raw_x = (int16_t)(_buffer[0] << 8 | _buffer[1]);
    int16_t raw_y = (int16_t)(_buffer[2] << 8 | _buffer[3]);
    int16_t raw_z = (int16_t)(_buffer[4] << 8 | _buffer[5]);

    float ax = raw_x / ACCEL_SENSITIVITY;
    float ay = raw_y / ACCEL_SENSITIVITY;
    float az = raw_z / ACCEL_SENSITIVITY;

    roll  = atan2(ay, az) * RAD_TO_DEG;
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
}