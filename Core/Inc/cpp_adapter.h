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

// --- Functions callable from C files ---

/**
 * @brief Initialize the global IMU object.
 * 
 * This wrapper creates the underlying C++ MPU6050 instance
 * and calls its initialization method. It should be called
 * once at system startup.
 */
void imuInit();

/**
 * @brief Periodic IMU processtick.
 * 
 * This function handles the IMU data cycle:
 * 1. Triggers DMA read
 * 2. Waits for Semaphore
 * 3. Processes Data
 * 4. Prints Telemetry
 * 
 * It is intended to be called from a FreeRTOS task.
 */
void imuTick();



#ifdef __cplusplus
}
#endif

#endif // CPP_ADAPTER_H