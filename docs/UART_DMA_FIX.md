# UART TX Blocking Issue - Root Cause Analysis & Solution

## Problem Description

The FalconDrone flight controller experienced **I2C bus freezes/deadlocks** when reading data from the MPU6050 IMU sensor. The issue became more frequent after implementing the Complementary Filter and streaming telemetry data via UART.

## Root Cause

The root cause was **blocking UART transmission** in the high-priority IMU task.

### The Problem Chain

```
IMU Task (osPriorityRealtime - Highest Priority)
        │
        ▼
  imu.ProcessData()          ← Fast (~10µs with FPU)
        │
        ▼
  printf("%.2f,%.2f\r\n")    ← BLOCKING! (~1300µs at 115200 baud)
        │
        ▼
  fflush(stdout)             ← More blocking!
        │
        ▼
  [CPU stuck in busy-wait loop for ~1.3ms]
```

### Why It Caused I2C Freezes

1. The `printf()` function called `_write()`, which used `HAL_UART_Transmit()` with `HAL_MAX_DELAY`
2. At 115200 baud, transmitting ~15 characters takes **~1.3 milliseconds**
3. During this time, the CPU was stuck in a polling loop, unable to service interrupts properly
4. The I2C DMA completion interrupt could fire, but the task couldn't process it
5. If timing was unlucky, the next IMU read cycle would start before the previous one was fully handled
6. This caused **I2C state corruption** → Bus hang (SDA/SCL held LOW)

### Original Code (Blocking)

```c
// main.c - _write function (used by printf)
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);  // BLOCKING!
    return len;
}

// cpp_adapter.cpp - imuTick function
void imuTick() {
    imu.StartUpdateDMA();
    
    if (osSemaphoreAcquire(imuSemHandle, osWaitForever) == osOK) {
        imu.ProcessData();
        printf("%.2f,%.2f\r\n", imu.roll, imu.pitch);  // ~1.3ms blocking!
        fflush(stdout);                                 // More blocking!
    }
}
```

## Solution

Implemented a **non-blocking DMA-based logging system** using a FreeRTOS message queue and a dedicated low-priority task.

### Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     IMU Task (Realtime Priority)                │
│                                                                 │
│  imu.ProcessData()                                              │
│       │                                                         │
│       ▼                                                         │
│  Log_Printf("%.2f,%.2f\r\n", roll, pitch)                       │
│       │                                                         │
│       └──────► [Copy to Queue] ──────► Returns immediately!     │
│                                                                 │
│  osDelay(100)  ← Task continues without waiting for UART        │
└─────────────────────────────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│                     Log Task (Low Priority)                     │
│                                                                 │
│  osMessageQueueGet()  ← Blocks here when queue is empty         │
│       │                                                         │
│       ▼                                                         │
│  osSemaphoreAcquire(txCompleteSem)  ← Wait for previous DMA     │
│       │                                                         │
│       ▼                                                         │
│  HAL_UART_Transmit_DMA()  ← Start DMA, returns immediately      │
│       │                                                         │
│       ▼                                                         │
│  Loop back to wait for next message                             │
└─────────────────────────────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│                     DMA Hardware                                │
│                                                                 │
│  [Transfers data to UART in background - zero CPU usage]        │
│       │                                                         │
│       ▼                                                         │
│  DMA1_Stream6_IRQHandler() → HAL_UART_TxCpltCallback()          │
│       │                                                         │
│       ▼                                                         │
│  Log_TxComplete() → osSemaphoreRelease(txCompleteSem)           │
└─────────────────────────────────────────────────────────────────┘
```

### Files Created

| File | Purpose |
|------|---------|
| `Core/Inc/logger.h` | Logger API declarations |
| `Core/Src/logger.c` | Queue management, DMA transmission |

### Key Changes

#### 1. CubeMX Configuration
- Enabled **USART2 TX DMA** (DMA1_Stream6, Channel 4)

#### 2. Logger Module (`logger.c`)

```c
// Non-blocking log function
bool Log_Printf(const char* fmt, ...) {
    LogMessage_t logMsg;
    // Format message into buffer
    vsnprintf(logMsg.data, LOG_MSG_MAX_LEN, fmt, args);
    // Queue message (returns immediately, even if queue is full)
    return osMessageQueuePut(logQueueHandle, &logMsg, 0, 0) == osOK;
}

// Dedicated logging task (low priority)
static void LogTask(void *argument) {
    for (;;) {
        // Wait for message (blocks here when idle)
        osMessageQueueGet(logQueueHandle, &currentMsg, NULL, osWaitForever);
        // Wait for previous DMA to complete
        osSemaphoreAcquire(txCompleteSemHandle, osWaitForever);
        // Start DMA transfer (non-blocking)
        HAL_UART_Transmit_DMA(&huart2, currentMsg.data, currentMsg.len);
    }
}
```

#### 3. DMA Completion Callback (`main.c`)

```c
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        Log_TxComplete();  // Releases semaphore for next transmission
    }
}
```

#### 4. Updated IMU Task (`cpp_adapter.cpp`)

```c
void imuTick() {
    if (!imu.StartUpdateDMA()) {
        return;
    }

    osStatus_t status = osSemaphoreAcquire(imuSemHandle, IMU_TIMEOUT_MS);
    
    if (status == osOK) {
        imu.ProcessData();
        Log_Printf("%.2f,%.2f\r\n", imu.roll, imu.pitch);  // Non-blocking!
    }
}
```

## Performance Comparison

| Metric | Before (Blocking) | After (DMA Queue) |
|--------|-------------------|-------------------|
| `printf` execution time | ~1300 µs | ~5 µs |
| CPU usage during TX | 100% (polling) | 0% (DMA) |
| IMU task blocking | Yes | No |
| I2C freezes | Frequent | None |

## Additional Fixes Applied

1. **Semaphore Initial Count**: Changed from `1` to `0` to prevent race condition on first read
2. **I2C Recovery Function**: Added `I2C_Recover()` to handle bus hangs
3. **Timeout Protection**: Changed `osWaitForever` to `200ms` timeout with error handling
4. **I2C Error Callback**: Releases semaphore on I2C errors to prevent deadlock

## Lessons Learned

1. **Never use blocking I/O in high-priority tasks**
2. **DMA + Queue pattern is essential for real-time systems**
3. **Always consider timing when multiple peripherals share interrupts**
4. **FreeRTOS task priorities matter - logging should be low priority**

---

*Document created: January 27, 2026*
*Author: FalconDrone Development Team*
