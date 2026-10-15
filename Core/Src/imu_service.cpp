/**
 * @file imu_service.cpp
 * @brief IMU service: task loop, DMA signalling, recovery and sample hand-off
 *        around the C++ MPU6050 driver and complementary filter.
 *
 * @author Yedidya Schwartz
 */

#include "imu_service.h"
#include "MPU6050.h"
#include "attitude_filter.h"
#include "i2c.h"        /* I2C_Recover — board-specific pins */
#include "logger.h"
#include "watchdog.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "cmsis_os2.h"

namespace {

/* A 14-byte read at 400 kHz takes ~0.4 ms; anything near a full period
 * means the transfer is not coming back. */
constexpr uint32_t kDmaTimeoutMs = 5;

/* Gaps longer than this (calibration, repeated bus failures) skip gyro
 * integration for one step rather than integrating a stale rate. */
constexpr uint32_t kMaxIntegrateGapMs = 200;

constexpr float kFilterAlpha = 0.98f;

/* Constant-initialized — no constructor runs before main(). */
MPU6050 s_imu;
ComplementaryFilter s_filter(kFilterAlpha);

I2C_HandleTypeDef *s_hi2c = nullptr;

/* DMA done / error → IMU task. Given from the I2C ISR. */
StaticSemaphore_t s_dma_sem_buf;
SemaphoreHandle_t s_dma_sem = nullptr;
volatile bool s_dma_error = false;

/* Single-slot mailbox IMU task → ControlTask. Overwritten each period, so
 * the reader always gets the newest complete sample, copied by value. */
StaticQueue_t s_mailbox_buf;
uint8_t s_mailbox_storage[sizeof(IMUData)];
QueueHandle_t s_mailbox = nullptr;

/* Set to PENDING by imuRequestCalibration(), advanced by the IMU task. A single
 * enum word = atomic load/store on Cortex-M4 so a plain volatile is sufficient. */
volatile ImuCalState s_cal_state = IMU_CAL_IDLE;

uint32_t s_last_sample_ms = 0;

/* Stop any in-flight DMA and clock the bus free. Used after a timeout or a
 * bus error — the HAL error path aborts DMA itself, a timeout does not. */
void recover_bus()
{
    if (s_hi2c->hdmarx != nullptr) {
        HAL_DMA_Abort(s_hi2c->hdmarx);
    }
    I2C_Recover(s_hi2c);
}

void run_calibration()
{
    s_cal_state = IMU_CAL_RUNNING;
    Log_Print("[IMU] Calibrating gyro — hold still 1s\r\n");
    if (s_imu.Calibrate()) {
        const GyroBias &b = s_imu.Bias();
        Log_Printf("[IMU] Bias: x=%+.2f y=%+.2f z=%+.2f dps\r\n", b.x, b.y, b.z);
        s_cal_state = IMU_CAL_OK;
    } else {
        Log_Print("[IMU] Calibration FAILED — board moved or I2C error\r\n");
        s_cal_state = IMU_CAL_FAILED;
    }
}

/* One read → fuse → publish cycle for the period that started at slot_ms. */
void sample_once(uint32_t slot_ms)
{
    xSemaphoreTake(s_dma_sem, 0);   /* drop a stale give from a late completion */
    s_dma_error = false;

    if (!s_imu.StartRead()) {
        /* HAL refused: state READY but bus stuck (BUSY flag, SDA low) or a
         * previous transfer never finished. Recover, retry next period. */
        recover_bus();
        return;
    }

    if (xSemaphoreTake(s_dma_sem, pdMS_TO_TICKS(kDmaTimeoutMs)) != pdTRUE) {
        Log_Print("[IMU] DMA timeout\r\n");
        recover_bus();
        return;
    }

    if (s_dma_error) {
        /* NACK / bus error: the buffer is partial or stale — never feed it
         * to the filter or the control loop. */
        Log_Printf("[IMU] I2C error 0x%lx\r\n", (unsigned long)HAL_I2C_GetError(s_hi2c));
        recover_bus();
        return;
    }

    const ImuSample raw = s_imu.Decode();

    const uint32_t gap_ms = slot_ms - s_last_sample_ms;
    const float dt = (gap_ms <= kMaxIntegrateGapMs) ? gap_ms / 1000.0f : 0.0f;
    s_last_sample_ms = slot_ms;
    s_filter.Update(raw, dt);

    IMUData out;
    out.t_ms   = slot_ms;
    out.roll   = s_filter.Roll();
    out.pitch  = s_filter.Pitch();
    out.gyro_x = raw.gx;
    out.gyro_y = raw.gy;
    out.gyro_z = raw.gz;
    xQueueOverwrite(s_mailbox, &out);
}

} // namespace

extern "C" void imuCreate(I2C_HandleTypeDef *hi2c)
{
    s_hi2c = hi2c;
    s_dma_sem = xSemaphoreCreateBinaryStatic(&s_dma_sem_buf);
    s_mailbox = xQueueCreateStatic(1, sizeof(IMUData), s_mailbox_storage, &s_mailbox_buf);
}

extern "C" void imuTaskRun(void)
{
    /* An MCU reset (RESET button, st-flash) does not power-cycle the sensor.
     * If the reset landed mid-transfer the MPU6050 can still be holding SDA
     * low, which leaves the I2C peripheral BUSY forever. Always clock the
     * bus free before the first transaction. */
    I2C_Recover(s_hi2c);

    if (s_imu.Init(s_hi2c)) {
        Log_Print("[IMU] Initialization Success!\r\n");
    } else {
        Log_Print("[IMU] Initialization FAILED!\r\n");
    }

    uint32_t next = osKernelGetTickCount();
    s_last_sample_ms = next;

    for (;;) {
        /* Deadline (2 s) covers the ~1 s calibration below. */
        Watchdog_CheckIn(WDG_IMU);

        if (s_cal_state == IMU_CAL_PENDING) {
            /* Bus is idle here (no DMA in flight), so Calibrate()'s blocking
             * reads are safe. */
            run_calibration();
        } else {
            sample_once(next);
        }

        /* Fixed-rate schedule: wake on exact multiples of the period so the
         * filter's dt is exact. If we fell behind (calibration takes ~1 s),
         * resync instead of bursting to catch up. */
        next += IMU_PERIOD_MS;
        const uint32_t now = osKernelGetTickCount();
        if ((int32_t)(next - now) <= 0) {
            next = now + IMU_PERIOD_MS;
        }
        osDelayUntil(next);
    }
}

extern "C" bool imuWaitSample(IMUData *out, uint32_t timeout_ms)
{
    return xQueueReceive(s_mailbox, out, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

extern "C" void imuRequestCalibration(void)
{
    s_cal_state = IMU_CAL_PENDING;
}

extern "C" ImuCalState imuGetCalState(void)
{
    return s_cal_state;
}

extern "C" void imuOnI2cRxComplete(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != s_hi2c || s_dma_sem == nullptr) return;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_dma_sem, &woken);
    portYIELD_FROM_ISR(woken);
}

extern "C" void imuOnI2cError(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != s_hi2c || s_dma_sem == nullptr) return;
    s_dma_error = true;
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_dma_sem, &woken);
    portYIELD_FROM_ISR(woken);
}
