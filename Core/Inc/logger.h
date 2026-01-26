/**
 * @file logger.h
 * @brief Non-blocking DMA-based logging system using FreeRTOS Queue.
 * 
 * This module provides a thread-safe, non-blocking logging mechanism
 * that uses a dedicated task to handle UART DMA transmissions.
 * 
 * @author Yedidya Schwartz
 * @date 2026
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/** Maximum length of a single log message */
#define LOG_MSG_MAX_LEN     64

/** Number of messages the queue can hold */
#define LOG_QUEUE_SIZE      16

/**
 * @brief Initialize the logging system.
 * 
 * Creates the message queue and logging task.
 * Must be called before using Log_Print().
 */
void Log_Init(void);

/**
 * @brief Queue a message for transmission (non-blocking).
 * 
 * This function copies the message to the queue and returns immediately.
 * The actual UART transmission happens in a separate low-priority task.
 * 
 * @param msg Null-terminated string to log.
 * @return true if message was queued, false if queue is full.
 */
bool Log_Print(const char* msg);

/**
 * @brief Queue a formatted message (printf-style).
 * 
 * @param fmt Format string (printf-style).
 * @param ... Variable arguments.
 * @return true if message was queued, false if queue is full.
 */
bool Log_Printf(const char* fmt, ...);

/**
 * @brief Signal that DMA transmission is complete.
 * 
 * Call this from HAL_UART_TxCpltCallback().
 */
void Log_TxComplete(void);

#ifdef __cplusplus
}
#endif

#endif /* LOGGER_H_ */
