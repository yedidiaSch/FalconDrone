/**
 * @file ICM20948.cpp
 * @brief DMA-based data acquisition for the ICM-20948.
 *
 * Register access is banked: REG_BANK_SEL (0x7F, present in every bank)
 * picks bank 0-3 in bits [5:4]. The sensor data and power registers are in
 * bank 0, the sample-rate / range / filter registers in bank 2. Init() ends
 * in bank 0 and nothing else changes banks, so the per-period DMA read needs
 * no bank write.
 *
 * Bus recovery after an MCU reset is the caller's job (see imu_service.cpp).
 *
 * @author Yedidya Schwartz
 */

#include "ICM20948.h"
#include "cmsis_os2.h"
#include <climits>

namespace {

/* GYRO_CONFIG_1 / ACCEL_CONFIG: [5:3] DLPFCFG, [2:1] FS_SEL, [0] FCHOICE.
 * FCHOICE=1 enables the DLPF. DLPFCFG=3 → gyro 51.2 Hz / accel 50.4 Hz 3 dB
 * bandwidth, the closest match to the MPU6050's 42 Hz DLPF_CFG=3 setting the
 * gains were tuned with. FS_SEL=0 → +/-250 dps, +/-2 g. */
constexpr uint8_t kDlpfCfg  = 3;
constexpr uint8_t kGyroCfg1 = (kDlpfCfg << 3) | (0 << 1) | 1;
constexpr uint8_t kAccelCfg = (kDlpfCfg << 3) | (0 << 1) | 1;

constexpr uint8_t kPwr1Reset = 0x80;   /**< DEVICE_RESET */
constexpr uint8_t kPwr1Run   = 0x01;   /**< SLEEP=0, CLKSEL=1 (PLL when ready) */

}  // namespace

bool ICM20948::WriteReg(uint16_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(_hi2c, kDeviceAddr, reg, 1, &value, 1, 100) == HAL_OK;
}

bool ICM20948::SelectBank(uint8_t bank)
{
    return WriteReg(kRegBankSel, (uint8_t)(bank << 4));
}

bool ICM20948::Init(I2C_HandleTypeDef* hi2c)
{
    _hi2c = hi2c;

    /* Bank state survives an MCU reset (the chip isn't power-cycled). */
    if (!SelectBank(0)) return false;

    uint8_t id = 0;
    if (HAL_I2C_Mem_Read(_hi2c, kDeviceAddr, kRegWhoAmI, 1, &id, 1, 100) != HAL_OK
        || id != kWhoAmI) {
        return false;
    }

    /* Full reset to known defaults (also selects bank 0), then wake. The
     * gyro needs ~35 ms to start after leaving sleep. */
    if (!WriteReg(kRegPwrMgmt1, kPwr1Reset)) return false;
    osDelay(50);
    if (!WriteReg(kRegPwrMgmt1, kPwr1Run)) return false;
    if (!WriteReg(kRegPwrMgmt2, 0x00)) return false;   /* accel + gyro all axes on */
    osDelay(50);

    /* Bank 2: output data rates at maximum (gyro 1.1 kHz, accel 1.125 kHz);
     * the 100 Hz reader always gets the newest filtered sample. */
    if (!SelectBank(2)) return false;
    const bool ok = WriteReg(kRegGyroSmplrtDiv, 0)
                 && WriteReg(kRegGyroConfig1, kGyroCfg1)
                 && WriteReg(kRegAccelSmplrtDiv1, 0)
                 && WriteReg(kRegAccelSmplrtDiv2, 0)
                 && WriteReg(kRegAccelConfig, kAccelCfg);

    /* Back to bank 0 even if a config write failed. */
    return SelectBank(0) && ok;
}

/**
 * Same procedure and limits as MPU6050::Calibrate(): 100 gyro reads 10 ms
 * apart; the old bias is kept if half the reads fail or any axis moves more
 * than 10 dps peak-to-peak.
 */
bool ICM20948::Calibrate()
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

bool ICM20948::StartRead()
{
    return HAL_I2C_Mem_Read_DMA(_hi2c, kDeviceAddr, kRegAccelXoutH, 1,
                                _buffer, kBlockSize) == HAL_OK;
}

ImuSample ICM20948::Decode() const
{
    /* Big-endian, high byte first, accel X Y Z then gyro X Y Z. */
    const int16_t raw_ax = (int16_t)(_buffer[0] << 8 | _buffer[1]);
    const int16_t raw_ay = (int16_t)(_buffer[2] << 8 | _buffer[3]);
    const int16_t raw_az = (int16_t)(_buffer[4] << 8 | _buffer[5]);
    const int16_t raw_gx = (int16_t)(_buffer[6] << 8 | _buffer[7]);
    const int16_t raw_gy = (int16_t)(_buffer[8] << 8 | _buffer[9]);
    const int16_t raw_gz = (int16_t)(_buffer[10] << 8 | _buffer[11]);

    ImuSample s;
    s.ax = raw_ax / kAccelLsbPerG;
    s.ay = raw_ay / kAccelLsbPerG;
    s.az = raw_az / kAccelLsbPerG;
    s.gx = raw_gx / kGyroLsbPerDps - _bias.x;
    s.gy = raw_gy / kGyroLsbPerDps - _bias.y;
    s.gz = raw_gz / kGyroLsbPerDps - _bias.z;
    return s;
}
