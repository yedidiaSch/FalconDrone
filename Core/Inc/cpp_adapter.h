/**
 * @file cpp_adapter.h
 * @brief C to C++ Adapter Layer
 * 
 * This file declares the C-compatible functions that allow
 * the C-based `main.c` and `freertos.c` to interface with the
 * C++ MPU6050 driver classes.
 * 
 * @author Yedidya Schwartz
 * @date 2026-01-12
 */

#ifndef CPP_ADAPTER_H
#define CPP_ADAPTER_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Snapshot of the latest IMU computation, copied out atomically.
 *
 * Roll/pitch are in degrees (complementary-filter fused).
 * Gyro rates are in degrees/second (raw scaled).
 */
typedef struct {
    float roll;
    float pitch;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} IMUData;

// --- Functions callable from C files ---

/**
 * @brief Initialize the global IMU object.
 */
void imuInit();

/**
 * @brief Periodic IMU process tick — DMA read, wait, fuse, signal.
 *
 * On success, releases imuDataReadySemHandle to wake the control task.
 */
void imuTick();

/**
 * @brief Copy the latest IMU values into `out` under a critical section.
 *
 * Safe to call from any task. Cheap (~5 word copy, interrupts off briefly).
 */
void imuGetSnapshot(IMUData *out);

#ifdef __cplusplus
}
#endif

#endif // CPP_ADAPTER_H