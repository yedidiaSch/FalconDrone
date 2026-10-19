/**
 * @file logger.c
 * @brief Implementation of non-blocking DMA-based logging system.
 * 
 * @author Yedidya Schwartz
 */

#include "logger.h"
#include "usart.h"
#include "cmsis_os2.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* External handles - created in freertos.c */
extern osMessageQueueId_t logQueueHandle;
extern osSemaphoreId_t txCompleteSemHandle;

/* Current message being transmitted */
static LogMessage_t currentMsg;

/**
 * @brief Process one message from the queue.
 * 
 * Called by LogTask in freertos.c. Blocks until message available.
 */
void Log_ProcessQueue(void)
{
    /* Wait for the previous DMA to complete FIRST — the DMA reads straight
     * from currentMsg, so pulling the next message into it while a transfer
     * is still running corrupts the line on the wire. */
    osSemaphoreAcquire(txCompleteSemHandle, osWaitForever);

    /* Wait for a message in the queue (blocks here when idle) */
    if (osMessageQueueGet(logQueueHandle, &currentMsg, NULL, osWaitForever) == osOK)
    {
        /* Start DMA transmission */
        HAL_UART_Transmit_DMA(&huart2, (uint8_t*)currentMsg.data, currentMsg.len);

        /* Semaphore will be released by Log_TxComplete() when DMA finishes */
    }
    else
    {
        /* Nothing was sent — hand the semaphore back for the next call. */
        osSemaphoreRelease(txCompleteSemHandle);
    }
}

/**
 * @brief Queue a message for transmission.
 */
bool Log_Print(const char* msg)
{
    LogMessage_t logMsg;
    
    /* Copy message, ensuring null termination */
    size_t len = strlen(msg);
    if (len >= LOG_MSG_MAX_LEN) {
        len = LOG_MSG_MAX_LEN - 1;
    }
    
    memcpy(logMsg.data, msg, len);
    logMsg.data[len] = '\0';
    logMsg.len = (uint16_t)len;
    
    /* Try to queue the message (don't wait if full - drop message) */
    if (osMessageQueuePut(logQueueHandle, &logMsg, 0, 0) == osOK) {
        return true;
    }
    
    return false;  /* Queue was full */
}

/**
 * @brief Queue a formatted message.
 */
bool Log_Printf(const char* fmt, ...)
{
    LogMessage_t logMsg;
    va_list args;
    
    va_start(args, fmt);
    int len = vsnprintf(logMsg.data, LOG_MSG_MAX_LEN, fmt, args);
    va_end(args);
    
    if (len < 0) {
        return false;
    }
    
    if (len >= LOG_MSG_MAX_LEN) {
        len = LOG_MSG_MAX_LEN - 1;
    }
    
    logMsg.len = (uint16_t)len;
    
    /* Try to queue the message */
    if (osMessageQueuePut(logQueueHandle, &logMsg, 0, 0) == osOK) {
        return true;
    }
    
    return false;
}

/**
 * @brief Signal DMA completion.
 */
void Log_TxComplete(void)
{
    osSemaphoreRelease(txCompleteSemHandle);
}
