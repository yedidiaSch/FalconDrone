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

/* Message structure for the queue */
typedef struct {
    char data[LOG_MSG_MAX_LEN];
    uint16_t len;
} LogMessage_t;

/* Queue and Task handles */
static osMessageQueueId_t logQueueHandle;
static osThreadId_t logTaskHandle;
static osSemaphoreId_t txCompleteSemHandle;

/* Current message being transmitted */
static LogMessage_t currentMsg;

/* Task attributes */
static const osThreadAttr_t logTask_attributes = {
    .name = "LogTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityLow,  /* Low priority - doesn't block critical tasks */
};

static const osSemaphoreAttr_t txCompleteSem_attributes = {
    .name = "txCompleteSem"
};

/* Forward declaration */
static void LogTask(void *argument);

/**
 * @brief Initialize the logging system.
 */
void Log_Init(void)
{
    /* Create the message queue */
    logQueueHandle = osMessageQueueNew(LOG_QUEUE_SIZE, sizeof(LogMessage_t), NULL);
    
    /* Create semaphore for DMA completion (starts at 1 - ready to transmit) */
    txCompleteSemHandle = osSemaphoreNew(1, 1, &txCompleteSem_attributes);
    
    /* Create the logging task */
    logTaskHandle = osThreadNew(LogTask, NULL, &logTask_attributes);
}

/**
 * @brief The logging task - processes queued messages.
 */
static void LogTask(void *argument)
{
    (void)argument;
    
    for (;;)
    {
        /* Wait for a message in the queue (blocks here when idle) */
        if (osMessageQueueGet(logQueueHandle, &currentMsg, NULL, osWaitForever) == osOK)
        {
            /* Wait for previous DMA to complete */
            osSemaphoreAcquire(txCompleteSemHandle, osWaitForever);
            
            /* Start DMA transmission */
            HAL_UART_Transmit_DMA(&huart2, (uint8_t*)currentMsg.data, currentMsg.len);
            
            /* Semaphore will be released by Log_TxComplete() when DMA finishes */
        }
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
