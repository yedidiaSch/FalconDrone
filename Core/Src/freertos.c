/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cpp_adapter.h"
#include "logger.h"
#include "pwm_esc.h"
#include "pid.h"
#include "mixer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IMUTask */
osThreadId_t IMUTaskHandle;
const osThreadAttr_t IMUTask_attributes = {
  .name = "IMUTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for LogTask */
osThreadId_t LogTaskHandle;
const osThreadAttr_t LogTask_attributes = {
  .name = "LogTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Definitions for logQueue */
osMessageQueueId_t logQueueHandle;
const osMessageQueueAttr_t logQueue_attributes = {
  .name = "logQueue"
};
/* Definitions for imuSem */
osSemaphoreId_t imuSemHandle;
const osSemaphoreAttr_t imuSem_attributes = {
  .name = "imuSem"
};
/* Definitions for imuDataReadySem — IMU → ControlTask handoff */
osSemaphoreId_t imuDataReadySemHandle;
const osSemaphoreAttr_t imuDataReadySem_attributes = {
  .name = "imuDataReadySem"
};
/* Definitions for txCompleteSem */
osSemaphoreId_t txCompleteSemHandle;
const osSemaphoreAttr_t txCompleteSem_attributes = {
  .name = "txCompleteSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartIMUTask(void *argument);
void StartLogTask(void *argument);
void StartControlTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of imuSem */
  imuSemHandle = osSemaphoreNew(1, 0, &imuSem_attributes);

  /* creation of imuDataReadySem */
  imuDataReadySemHandle = osSemaphoreNew(1, 0, &imuDataReadySem_attributes);

  /* creation of txCompleteSem */
  txCompleteSemHandle = osSemaphoreNew(1, 1, &txCompleteSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of logQueue */
  logQueueHandle = osMessageQueueNew (16, LOG_MSG_STRUCT_SIZE, &logQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of IMUTask */
  IMUTaskHandle = osThreadNew(StartIMUTask, NULL, &IMUTask_attributes);

  /* creation of LogTask */
  LogTaskHandle = osThreadNew(StartLogTask, NULL, &LogTask_attributes);

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartIMUTask */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void *argument)
{
  /* USER CODE BEGIN StartIMUTask */
  
  imuInit();

  /* 100 Hz IMU sample rate — matches the control loop. */
  for(;;)
  {
    imuTick();
    osDelay(10);
  }
  /* USER CODE END StartIMUTask */
}

/* USER CODE BEGIN Header_StartLogTask */
/**
* @brief Function implementing the LogTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLogTask */
void StartLogTask(void *argument)
{
  /* USER CODE BEGIN StartLogTask */
  /* Infinite loop */
  for(;;)
  {
    Log_ProcessQueue();
  }
  /* USER CODE END StartLogTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/*
 * ControlTask — Phase 3 attitude controller.
 *
 * Synchronous with the IMU: blocks on imuDataReadySem, runs once per
 * fresh sample. 100 Hz nominal (matches IMU loop).
 *
 * Pipeline each tick:
 *   1. Wait for fresh IMU sample (sem release from imuTick)
 *   2. Snapshot gyro rates + fused angles
 *   3. Three rate-mode PIDs (setpoint = 0 → self-leveling)
 *   4. Mixer → 4 motor pulse widths
 *   5. ESC_SetAllMotors
 *
 * SAFETY: base_throttle is hard-pinned to ESC_MIN_US. With props off you can
 * verify on the bench that motor outputs respond to tilting (one motor pair
 * ramps up, the opposite pair stays at min). Do NOT raise base_throttle
 * until PID gains have been bench-tuned and a tether is in place.
 */
void StartControlTask(void *argument)
{
    static PID_t pid_roll, pid_pitch, pid_yaw;

    /* Conservative starting gains — must be tuned on a single-axis rig.
     * out_limit (300 µs) caps each axis's authority so a runaway PID
     * cannot saturate the throttle range. */
    PID_Init(&pid_roll,  0.6f, 0.0f, 0.02f, /*i_limit*/200.0f, /*out_limit*/300.0f);
    PID_Init(&pid_pitch, 0.6f, 0.0f, 0.02f, 200.0f, 300.0f);
    PID_Init(&pid_yaw,   1.5f, 0.0f, 0.00f, 200.0f, 300.0f);

    ESC_Init();

    /* Hold idle pulse while ESCs run their startup beep sequence. */
    osDelay(ESC_ARM_DELAY_MS);
    Log_Print("ESC armed — control loop starting\r\n");

    uint32_t last_tick = osKernelGetTickCount();
    uint32_t log_div   = 0;   /* decimate telemetry to 50 Hz */

    for (;;)
    {
        /* Block on fresh IMU sample. 100 ms timeout = fail-safe to idle. */
        if (osSemaphoreAcquire(imuDataReadySemHandle, 100) != osOK) {
            ESC_SetAllMotors(ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US);
            PID_Reset(&pid_roll);
            PID_Reset(&pid_pitch);
            PID_Reset(&pid_yaw);
            continue;
        }

        const uint32_t now = osKernelGetTickCount();
        float dt = (float)(now - last_tick) / 1000.0f;
        last_tick = now;
        if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;  /* sanity clamp */

        IMUData s;
        imuGetSnapshot(&s);

        /* Rate-mode setpoints. Replace with RC stick input later. */
        const float roll_rate_sp  = 0.0f;
        const float pitch_rate_sp = 0.0f;
        const float yaw_rate_sp   = 0.0f;

        const float roll_out  = PID_Update(&pid_roll,  roll_rate_sp,  s.gyro_x, dt);
        const float pitch_out = PID_Update(&pid_pitch, pitch_rate_sp, s.gyro_y, dt);
        const float yaw_out   = PID_Update(&pid_yaw,   yaw_rate_sp,   s.gyro_z, dt);

        /* SAFETY pin — see header comment before changing. */
        const float base_throttle = (float)ESC_MIN_US;

        uint16_t motors[4];
        Mixer_Compute(base_throttle, roll_out, pitch_out, yaw_out,
                      ESC_MIN_US, ESC_MAX_US, motors);

        ESC_SetAllMotors(motors[0], motors[1], motors[2], motors[3]);

        /* Telemetry — one CSV line every 2 ticks (≈50 Hz).
         * Format: t_ms,roll,pitch,gx,gy,gz,m1,m2,m3,m4 */
        if ((++log_div & 0x1) == 0u) {
            Log_Printf("%lu,%.1f,%.1f,%.1f,%.1f,%.1f,%u,%u,%u,%u\r\n",
                       (unsigned long)now,
                       s.roll,   s.pitch,
                       s.gyro_x, s.gyro_y, s.gyro_z,
                       motors[0], motors[1], motors[2], motors[3]);
        }
    }
}

/* USER CODE END Application */

