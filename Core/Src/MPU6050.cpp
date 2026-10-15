/**
 * @file MPU6050.cpp
 * @brief Implementation of DMA-based data acquisition for MPU6050.
 *
 * @author Yedidya Schwartz
 */

#include "MPU6050.h"
#include "cmsis_os2.h"
#include <climits>

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
 * Bus recovery after an MCU reset is the caller's job (see imu_service.cpp) —
 * it is board-specific (pins), this driver is not.
 *
 * @return true if both writes succeeded.
 */
bool MPU6050::Init(I2C_HandleTypeDef* hi2c) {
    _hi2c = hi2c;

    uint8_t wake_up_data = 0;
    if (HAL_I2C_Mem_Write(_hi2c, kDeviceAddr, 0x6B, 1, &wake_up_data, 1, 100) != HAL_OK) {
        return false;
    }

    uint8_t dlpf_cfg = 0x03;  /* ~42 Hz LPF on accel + gyro */
    return (HAL_I2C_Mem_Write(_hi2c, kDeviceAddr, 0x1A, 1, &dlpf_cfg, 1, 100) == HAL_OK);
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
    constexpr int32_t MAX_SPAN_LSB = (int32_t)(CAL_MAX_SPAN_DPS * kGyroLsbPerDps);

    int32_t sum[3] = {0, 0, 0};
    int16_t lo[3]  = {INT16_MAX, INT16_MAX, INT16_MAX};
    int16_t hi[3]  = {INT16_MIN, INT16_MIN, INT16_MIN};
    int valid = 0;
    uint8_t gbuf[6];

    for (int i = 0; i < N; ++i) {
        if (HAL_I2C_Mem_Read(_hi2c, kDeviceAddr, kRegGyroXoutH, 1, gbuf, 6, 50) == HAL_OK) {
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

    _bias.x = (float)sum[0] / valid / kGyroLsbPerDps;
    _bias.y = (float)sum[1] / valid / kGyroLsbPerDps;
    _bias.z = (float)sum[2] / valid / kGyroLsbPerDps;
    return true;
}

/**
 * @brief Starts the DMA transfer for sensor registers.
 * Reads registers 0x3B to 0x48 (Accel, Temp, Gyro).
 * The HAL itself returns HAL_BUSY when a transfer is already running or the
 * bus is stuck, so no separate state check is needed here.
 */
bool MPU6050::StartRead() {
    return HAL_I2C_Mem_Read_DMA(_hi2c, kDeviceAddr, kRegAccelXoutH, 1,
                                _buffer, kBlockSize) == HAL_OK;
}

/**
 * @brief Converts the raw DMA buffer into physical units.
 */
ImuSample MPU6050::Decode() const {
    // --- Accelerometer Raw Data ---
    const int16_t raw_ax = (int16_t)(_buffer[0] << 8 | _buffer[1]);
    const int16_t raw_ay = (int16_t)(_buffer[2] << 8 | _buffer[3]);
    const int16_t raw_az = (int16_t)(_buffer[4] << 8 | _buffer[5]);

    // --- Gyroscope Raw Data (Starting from index 8, skipping Temperature) ---
    const int16_t raw_gx = (int16_t)(_buffer[8] << 8  | _buffer[9]);
    const int16_t raw_gy = (int16_t)(_buffer[10] << 8 | _buffer[11]);
    const int16_t raw_gz = (int16_t)(_buffer[12] << 8 | _buffer[13]);

    ImuSample s;
    s.ax = raw_ax / kAccelLsbPerG;
    s.ay = raw_ay / kAccelLsbPerG;
    s.az = raw_az / kAccelLsbPerG;

    // Gyro in Degrees Per Second (DPS), with per-axis DC bias removed.
    s.gx = raw_gx / kGyroLsbPerDps - _bias.x;
    s.gy = raw_gy / kGyroLsbPerDps - _bias.y;
    s.gz = raw_gz / kGyroLsbPerDps - _bias.z;
    return s;
}
