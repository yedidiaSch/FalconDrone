/**
 * @file ICM20948.h
 * @brief Driver class for the TDK ICM-20948 IMU (accel + gyro; the AK09916
 *        magnetometer inside is not used).
 *
 * Same interface as MPU6050 so imu_service.cpp can use either. Wired as on
 * hardware/carrier: I2C1 through a PCA9306 to the chip's 1.8 V I/O, AD0 high
 * (address 0x69), INT1 open (polled).
 *
 * Reference: TDK DS-000189 rev 1.3, sections 7-9 (register bank map).
 *
 * @author Yedidya Schwartz
 * @date 2026
 */

#ifndef ICM20948_H_
#define ICM20948_H_

#include "stm32f4xx_hal.h"
#include "imu_types.h"
#include <cstdint>

/**
 * @class ICM20948
 * @brief Handles ICM-20948 sensor operations.
 *
 * Constant-initialized (no runtime constructor), so a static instance is
 * safe before main(). The I2C handle is supplied in Init().
 */
class ICM20948 {
public:
    constexpr ICM20948() = default;

    /**
     * @brief Checks WHO_AM_I, resets the chip, wakes it and configures the
     *        full scales and digital low-pass filters.
     *
     * Blocks for ~100 ms (reset and gyro start-up). Leaves register bank 0
     * selected, which StartRead() and Calibrate() rely on.
     *
     * @param hi2c I2C handle the sensor is attached to.
     * @return true if the chip answered with the right ID and every write succeeded.
     */
    bool Init(I2C_HandleTypeDef* hi2c);

    /** @copydoc MPU6050::Calibrate */
    bool Calibrate();

    /**
     * @brief Starts a non-blocking DMA read of ACCEL_XOUT_H..GYRO_ZOUT_L.
     *
     * Completion is reported through the HAL I2C callbacks.
     * @return false if the HAL refused the transfer (bus busy or stuck).
     */
    bool StartRead();

    /** @brief Decode the buffer filled by the last completed DMA read. */
    ImuSample Decode() const;

    const GyroBias& Bias() const { return _bias; }

private:
    static constexpr uint16_t kDeviceAddr = (0x69 << 1); /**< AD0 high, HAL 8-bit form */
    static constexpr uint8_t  kWhoAmI     = 0xEA;

    /* Bank 0 */
    static constexpr uint16_t kRegWhoAmI    = 0x00;
    static constexpr uint16_t kRegPwrMgmt1  = 0x06;
    static constexpr uint16_t kRegPwrMgmt2  = 0x07;
    static constexpr uint16_t kRegAccelXoutH = 0x2D;   /**< accel X..Z, then gyro X..Z */
    static constexpr uint16_t kRegGyroXoutH  = 0x33;
    /* Bank 2 */
    static constexpr uint16_t kRegGyroSmplrtDiv  = 0x00;
    static constexpr uint16_t kRegGyroConfig1    = 0x01;
    static constexpr uint16_t kRegAccelSmplrtDiv1 = 0x10;
    static constexpr uint16_t kRegAccelSmplrtDiv2 = 0x11;
    static constexpr uint16_t kRegAccelConfig    = 0x14;
    /* Any bank */
    static constexpr uint16_t kRegBankSel = 0x7F;

    static constexpr uint16_t kBlockSize = 12;   /**< 6 Accel + 6 Gyro, no temperature */

    /* Gyro at full range so fast yaw or flips never clip; Decode() returns
     * dps either way, so the range does not change the PID gains (0.06 dps
     * per LSB is still far below the sensor noise). Accel stays at +/-2 g:
     * it is only the gravity reference for the attitude filter. */
    static constexpr float kAccelLsbPerG  = 16384.0f; /**< Range +/- 2g */
    static constexpr float kGyroLsbPerDps = 16.4f;    /**< Range +/- 2000dps */

    bool WriteReg(uint16_t reg, uint8_t value);
    bool SelectBank(uint8_t bank);

    I2C_HandleTypeDef* _hi2c = nullptr;
    uint8_t _buffer[kBlockSize] = {};   /**< DMA target (Accel, Gyro) */
    GyroBias _bias = {0.0f, 0.0f, 0.0f};
};

#endif // ICM20948_H_
