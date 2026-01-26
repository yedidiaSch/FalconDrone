/**
 * @file MPU6050.cpp
 * @brief Implementation of DMA-based data acquisition for MPU6050.
 * 
 * @author Yedidya Schwartz
 */

#include "MPU6050.h"

/**
 * @brief Constructs the MPU6050 driver instance.
 * @param hi2c I2C Handle
 * @param sem Semaphore Handle
 */
MPU6050::MPU6050(I2C_HandleTypeDef* hi2c, osSemaphoreId_t sem) 
    : roll(0.0f), pitch(0.0f), gyro_x(0.0f), gyro_y(0.0f), gyro_z(0.0f),
      _hi2c(hi2c), _sem(sem), _buffer{0}, _last_tick(0)
{
}

/**
 * @brief Wakes up the sensor.
 * @return true if communication succeeded.
 */
bool MPU6050::Init() {
    uint8_t wake_up_data = 0;
    // Wake up using standard blocking call (only happens once at start)
    return (HAL_I2C_Mem_Write(_hi2c, DEVICE_ADDR, 0x6B, 1, &wake_up_data, 1, 100) == HAL_OK);
}

/**
 * @brief Starts the DMA transfer for sensor registers.
 * Reads registers 0x3B to 0x48 (Accel, Temp, Gyro).
 * @return true if DMA started successfully, false if bus is busy.
 */
bool MPU6050::StartUpdateDMA() {
    // Check if I2C is ready before starting using HAL function
    if (HAL_I2C_GetState(_hi2c) != HAL_I2C_STATE_READY) {
        // Attempt I2C recovery
        I2C_Recover(_hi2c);
        return false;
    }
    
    // Start non-blocking DMA read
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(_hi2c, DEVICE_ADDR, 0x3B, 1, _buffer, MPU_BLOCK_SIZE);
    return (status == HAL_OK);
}

/**
 * @brief Computes physical values from the raw DMA buffer.
 */
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

    // Time Delta Calculation
    uint32_t current_tick = HAL_GetTick();
    float dt = (current_tick - _last_tick) / 1000.0f;
    _last_tick = current_tick;

    // Basic dt safety
    if(dt > 0.2f) dt = 0.0f;

    // Static Angle Calculation (Accelerometer only)
    float accel_roll  = atan2(ay, az) * RAD_TO_DEG;
    float accel_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

    // Complementary Filter
    roll  = ALPHA * (roll + gyro_x * dt) + (1.0f - ALPHA) * accel_roll;
    pitch = ALPHA * (pitch + gyro_y * dt) + (1.0f - ALPHA) * accel_pitch;
}