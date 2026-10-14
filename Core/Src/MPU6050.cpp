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
      gyro_bias_x(0.0f), gyro_bias_y(0.0f), gyro_bias_z(0.0f),
      _hi2c(hi2c), _sem(sem), _buffer{0}, _last_tick(0)
{
}

/**
 * @brief Wakes up the sensor and configures the digital low-pass filter.
 *
 * Default DLPF is wide-open (~256 Hz cutoff), which lets a lot of vibration
 * and electrical noise through — measured ~7-9 dps gyro noise stationary.
 * DLPF_CFG = 3 sets the on-chip LPF to ~42 Hz on both accel and gyro, which
 * is well above our 100 Hz control loop's Nyquist concerns and dramatically
 * cleans up the signal feeding into the PID derivative term.
 *
 * Reference: MPU-6000 register map, register 0x1A (CONFIG):
 *   bits [2:0] DLPF_CFG: 0=256Hz, 1=188Hz, 2=98Hz, 3=42Hz, 4=20Hz, 5=10Hz, 6=5Hz
 *
 * @return true if both writes succeeded.
 */
bool MPU6050::Init() {
    /* An MCU reset (RESET button, st-flash) does not power-cycle the sensor.
     * If the reset landed mid-transfer the MPU6050 can still be holding SDA
     * low, which leaves the I2C peripheral BUSY forever. Always clock the
     * bus free before the first transaction. */
    I2C_Recover(_hi2c);

    uint8_t wake_up_data = 0;
    if (HAL_I2C_Mem_Write(_hi2c, DEVICE_ADDR, 0x6B, 1, &wake_up_data, 1, 100) != HAL_OK) {
        return false;
    }

    uint8_t dlpf_cfg = 0x03;  /* ~42 Hz LPF on accel + gyro */
    return (HAL_I2C_Mem_Write(_hi2c, DEVICE_ADDR, 0x1A, 1, &dlpf_cfg, 1, 100) == HAL_OK);
}

/**
 * @brief Sample 100 gyro readings while the board is stationary and store
 *        the mean per axis as the DC bias offset.
 *
 * The MPU6050 has a per-unit zero-rate offset of typically ±2 dps. Without
 * correction it becomes a permanent setpoint error the PID has to fight.
 * Caller is responsible for ensuring the board is held still during the
 * ~1.05 s this function blocks.
 *
 * The previous bias is kept (and false returned) if too many I2C reads fail,
 * or if any axis swings more than CAL_MAX_SPAN_DPS peak-to-peak — that means
 * the board moved and the mean would bake the motion in as a fake bias.
 * Re-callable any time.
 */
bool MPU6050::Calibrate()
{
    osDelay(50);  /* let any in-flight motion settle */

    constexpr int N = 100;
    constexpr float CAL_MAX_SPAN_DPS = 10.0f;
    constexpr int32_t MAX_SPAN_LSB = (int32_t)(CAL_MAX_SPAN_DPS * GYRO_SENSITIVITY);

    int32_t sum[3] = {0, 0, 0};
    int16_t lo[3]  = {INT16_MAX, INT16_MAX, INT16_MAX};
    int16_t hi[3]  = {INT16_MIN, INT16_MIN, INT16_MIN};
    int valid = 0;
    uint8_t gbuf[6];

    for (int i = 0; i < N; ++i) {
        if (HAL_I2C_Mem_Read(_hi2c, DEVICE_ADDR, 0x43, 1, gbuf, 6, 50) == HAL_OK) {
            for (int a = 0; a < 3; ++a) {
                const int16_t v = (int16_t)((gbuf[2 * a] << 8) | gbuf[2 * a + 1]);
                sum[a] += v;
                if (v < lo[a]) lo[a] = v;
                if (v > hi[a]) hi[a] = v;
            }
            ++valid;
        }
        osDelay(10);  /* yields so LogTask can drain its queue during cal */
    }

    if (valid < N / 2) {
        return false;
    }
    for (int a = 0; a < 3; ++a) {
        if ((int32_t)hi[a] - (int32_t)lo[a] > MAX_SPAN_LSB) {
            return false;
        }
    }

    gyro_bias_x = (float)sum[0] / valid / GYRO_SENSITIVITY;
    gyro_bias_y = (float)sum[1] / valid / GYRO_SENSITIVITY;
    gyro_bias_z = (float)sum[2] / valid / GYRO_SENSITIVITY;
    return true;
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
    if (status != HAL_OK) {
        // HAL state can be READY while the bus itself is stuck (BUSY flag
        // set, SDA held low) — the read then fails with HAL_BUSY every tick.
        // Recover here too, or the IMU never comes back.
        I2C_Recover(_hi2c);
        return false;
    }
    return true;
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

    // Gyro in Degrees Per Second (DPS), with per-axis DC bias removed.
    gyro_x = raw_gx / GYRO_SENSITIVITY - gyro_bias_x;
    gyro_y = raw_gy / GYRO_SENSITIVITY - gyro_bias_y;
    gyro_z = raw_gz / GYRO_SENSITIVITY - gyro_bias_z;

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