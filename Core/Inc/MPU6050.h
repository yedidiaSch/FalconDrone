/**
 * @file MPU6050.h
 * @brief Driver class for MPU6050 IMU.
 *
 * Pure sensor I/O: register setup, gyro bias calibration, and a DMA burst
 * read decoded into physical units. Attitude fusion lives in
 * attitude_filter.h; task timing, bus recovery and the hand-off to the
 * control loop live in imu_service.cpp.
 *
 * @author Yedidya Schwartz
 * @date 2026
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "stm32f4xx_hal.h"
#include <cstdint>

/** One decoded sensor read. Accel in g, gyro in dps with bias removed. */
struct ImuSample {
    float ax, ay, az;
    float gx, gy, gz;
};

/** Per-axis gyro zero-rate offset in dps. */
struct GyroBias {
    float x, y, z;
};

/**
 * @class MPU6050
 * @brief Handles MPU6050 sensor operations.
 *
 * Constant-initialized (no runtime constructor), so a static instance is
 * safe before main(). The I2C handle is supplied in Init().
 */
class MPU6050 {
public:
    constexpr MPU6050() = default;

    /**
     * @brief Wakes the sensor and configures the digital low-pass filter.
     * @param hi2c I2C handle the sensor is attached to.
     * @return true if both register writes succeeded.
     */
    bool Init(I2C_HandleTypeDef* hi2c);

    /**
     * @brief Sample the gyroscope while stationary and store DC bias offsets.
     *
     * Blocks for ~1 second (100 samples * 10 ms). The board MUST be still.
     * Subsequent Decode() calls subtract the measured bias.
     *
     * @return true if the new bias was applied; false if the board moved
     *         during sampling or too many I2C reads failed (old bias kept).
     */
    bool Calibrate();

    /**
     * @brief Starts a non-blocking DMA read of ACCEL_XOUT_H..GYRO_ZOUT_L.
     *
     * Completion is reported through the HAL I2C callbacks.
     * @return false if the HAL refused the transfer (bus busy or stuck).
     */
    bool StartRead();

    /**
     * @brief Decode the buffer filled by the last completed DMA read.
     */
    ImuSample Decode() const;

    const GyroBias& Bias() const { return _bias; }

private:
    static constexpr uint16_t kDeviceAddr = (0x68 << 1); /**< AD0 low, HAL 8-bit form */
    static constexpr uint16_t kRegAccelXoutH = 0x3B;
    static constexpr uint16_t kRegGyroXoutH  = 0x43;
    static constexpr uint16_t kBlockSize     = 14;   /**< 6 Accel + 2 Temp + 6 Gyro */

    // Sensitivities for MPU6050 (Default settings)
    static constexpr float kAccelLsbPerG   = 16384.0f; /**< Range +/- 2g */
    static constexpr float kGyroLsbPerDps  = 131.0f;   /**< Range +/- 250dps */

    I2C_HandleTypeDef* _hi2c = nullptr;
    uint8_t _buffer[kBlockSize] = {};   /**< DMA target (Accel, Temp, Gyro) */
    GyroBias _bias = {0.0f, 0.0f, 0.0f};
};

#endif // MPU6050_H_
