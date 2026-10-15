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
#include "imu_service.h"
#include "i2c.h"
#include "logger.h"
#include "pwm_esc.h"
#include "pid.h"
#include "mixer.h"
#include "arming.h"
#include "watchdog.h"
#include "reset_info.h"
#include "gpio.h"
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
  .stack_size = 256 * 4,
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
  /* creation of txCompleteSem */
  txCompleteSemHandle = osSemaphoreNew(1, 1, &txCompleteSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* IMU service owns its DMA semaphore and sample mailbox. Created here so
   * they exist before any task can touch them. */
  imuCreate(&hi2c1);
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
  ResetInfo_Report();

  /* Watchdog supervisor + LD2 heartbeat (doubles as the arm-state
   * indicator: slow = DISARMED, medium = ARMING, fast = ARMED).
   * Runs at Normal priority, below IMU and Control, so if either of them
   * hogs the CPU this loop starves and the IWDG fires. */
  uint32_t last_toggle = osKernelGetTickCount();
  for(;;)
  {
    const uint32_t now = osKernelGetTickCount();
    Watchdog_Supervise(now);

    uint32_t blink_ms;
    switch (Arming_GetState()) {
    case ARM_STATE_ARMED:  blink_ms = 100;  break;
    case ARM_STATE_ARMING: blink_ms = 250;  break;
    default:               blink_ms = 1000; break;
    }
    if (now - last_toggle >= blink_ms) {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      last_toggle = now;
    }

    osDelay(WATCHDOG_SUPERVISE_PERIOD_MS);
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
  
  /* Fixed 100 Hz read / fuse / publish loop — never returns. */
  imuTaskRun();
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
 * Synchronous with the IMU: blocks on the IMU mailbox, runs once per
 * fresh sample. 100 Hz (IMU_PERIOD_MS).
 *
 * Pipeline each tick:
 *   1. Wait for the next IMU sample (by-value copy from the mailbox)
 *   2. dt from the sample timestamps (exact multiples of IMU_PERIOD_MS)
 *   3. Arming state machine (arming.c) — B1 / PC command / failsafes
 *   4. If ARMED: three rate-mode PIDs (setpoint = 0 → damps rotation; this
 *      does NOT self-level — there is no outer angle loop yet) → mixer.
 *      Otherwise: all motors at ESC_MIN_US and PID state cleared.
 *   5. ESC_SetAllMotors
 *
 * Boot state is DISARMED: motors stay at ESC_MIN_US until B1 is held 2 s.
 *
 * SAFETY: base_throttle = 1300 us is BENCH-TEST ONLY (props removed!).
 * With out_limit = 300 each axis can swing the motors symmetrically across
 * 1000-1600 us, which is required to observe both sides of the PID response
 * during gain tuning. ESC_MIN_US (1000) makes the response asymmetric
 * because the negative side gets clamped at idle.
 *
 * BEFORE flying with props ON: re-pin base_throttle to ESC_MIN_US, validate
 * gains on a single-axis tethered rig, then choose a hover-throttle base
 * (typically 1400-1600 depending on thrust-to-weight) for the actual flight.
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
    Log_Print("ESC ready — DISARMED. Hold B1 2 s to arm.\r\n");

    uint32_t last_sample_ms = 0;
    uint32_t log_div   = 0;   /* decimate telemetry to 50 Hz — 100 Hz overruns the queue */

    for (;;)
    {
        Watchdog_CheckIn(WDG_CONTROL);

        /* Watchdog bench test ('h' from the PC, Debug builds): stop checking
         * in while the motors are at idle. Expect an IWDG reset ~0.5 s later. */
        if (Watchdog_TestHangRequested() && Arming_GetState() == ARM_STATE_DISARMED) {
            Log_Print("[WDG] Test: ControlTask hanging on purpose\r\n");
            for (;;) {
                osDelay(1000);
            }
        }

        /* B1 (PC13) has an external pull-up: idle high, pressed = low. */
        const uint8_t b1_pressed =
            (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET);

        /* Block on fresh IMU sample. 100 ms timeout = fail-safe to idle
         * (Arming_Update disarms on a NULL sample while ARMED). */
        IMUData s;
        if (!imuWaitSample(&s, 100)) {
            Arming_Update(osKernelGetTickCount(), b1_pressed, NULL);
            ESC_SetAllMotors(ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US);
            PID_Reset(&pid_roll);
            PID_Reset(&pid_pitch);
            PID_Reset(&pid_yaw);
            continue;
        }

        /* Sample timestamps are scheduled slot times, so dt is exact —
         * 10 ms normally, 20 ms if the IMU dropped one read. */
        const uint32_t now = s.t_ms;
        float dt = (float)(now - last_sample_ms) / 1000.0f;
        last_sample_ms = now;
        if (dt <= 0.0f || dt > 0.1f) dt = IMU_PERIOD_MS / 1000.0f;  /* first sample / after a gap */

        const ArmState_t arm = Arming_Update(now, b1_pressed, &s);

        uint16_t motors[4] = { ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US };

        if (arm == ARM_STATE_ARMED) {
            /* Rate-mode setpoints. Replace with RC stick input later. */
            const float roll_rate_sp  = 0.0f;
            const float pitch_rate_sp = 0.0f;
            const float yaw_rate_sp   = 0.0f;

            const float roll_out  = PID_Update(&pid_roll,  roll_rate_sp,  s.gyro_x, dt);
            const float pitch_out = PID_Update(&pid_pitch, pitch_rate_sp, s.gyro_y, dt);
            const float yaw_out   = PID_Update(&pid_yaw,   yaw_rate_sp,   s.gyro_z, dt);

            /* BENCH-TEST base — props OFF. See header comment before changing. */
            const float base_throttle = 1300.0f;

            Mixer_Compute(base_throttle, roll_out, pitch_out, yaw_out,
                          ESC_MIN_US, ESC_MAX_US, motors);
        } else {
            /* Not armed: keep PIDs clean so arming starts from zero state. */
            PID_Reset(&pid_roll);
            PID_Reset(&pid_pitch);
            PID_Reset(&pid_yaw);
        }

        ESC_SetAllMotors(motors[0], motors[1], motors[2], motors[3]);

        /* Telemetry — one CSV line every 2 ticks (~50 Hz).
         * 100 Hz overruns the 16-deep log queue during motion bursts (~33%
         * loss observed). 50 Hz drains comfortably under UART 115200.
         * Format: t_ms,roll,pitch,gx,gy,gz,m1,m2,m3,m4,arm
         * arm: 0 = disarmed, 1 = armed, 2 = arming */
        if ((++log_div & 0x1) == 0u) {
            Log_Printf("%lu,%.1f,%.1f,%.1f,%.1f,%.1f,%u,%u,%u,%u,%u\r\n",
                       (unsigned long)now,
                       s.roll,   s.pitch,
                       s.gyro_x, s.gyro_y, s.gyro_z,
                       motors[0], motors[1], motors[2], motors[3],
                       (unsigned)arm);
        }
    }
}

/* USER CODE END Application */

