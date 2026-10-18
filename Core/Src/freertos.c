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
#include "flight_ctrl.h"
#include "flight_input.h"
#include "uart_cmd.h"
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
 * ControlTask — cascaded attitude controller (flight_ctrl.c).
 *
 * Synchronous with the IMU: blocks on the IMU mailbox, runs once per
 * fresh sample. 100 Hz (IMU_PERIOD_MS).
 *
 * Pipeline each tick:
 *   1. Wait for the next IMU sample (by-value copy from the mailbox)
 *   2. dt from the sample timestamps (exact multiples of IMU_PERIOD_MS)
 *   3. Arming state machine (arming.c) — B1 / PC command / failsafes
 *   4. PC line commands (uart_cmd.c) — throttle, mode, gains, trim
 *   5. If ARMED: FlightInput (throttle ramp) → FlightCtrl (angle loop in
 *      ANGLE mode → rate PIDs → mixer). Otherwise: motors at ESC_MIN_US,
 *      PID state and throttle cleared.
 *   6. ESC_SetAllMotors
 *
 * Boot state is DISARMED: motors stay at ESC_MIN_US until B1 is held 2 s.
 * Every arm starts at ESC_MIN_US; throttle then comes only from $t commands,
 * capped at THROTTLE_MAX_US and ramped at THROTTLE_SLEW_US_S (flight_cfg.h).
 */
void StartControlTask(void *argument)
{
    FlightCtrl_Init();
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
            const ArmState_t arm = Arming_Update(osKernelGetTickCount(), b1_pressed, NULL);
            UartCmd_Poll(arm);
            ESC_SetAllMotors(ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US);
            FlightCtrl_Idle(NULL);
            FlightInput_Reset();
            continue;
        }

        /* Sample timestamps are scheduled slot times, so dt is exact —
         * 10 ms normally, 20 ms if the IMU dropped one read. */
        const uint32_t now = s.t_ms;
        float dt = (float)(now - last_sample_ms) / 1000.0f;
        last_sample_ms = now;
        if (dt <= 0.0f || dt > 0.1f) dt = IMU_PERIOD_MS / 1000.0f;  /* first sample / after a gap */

        const ArmState_t arm = Arming_Update(now, b1_pressed, &s);
        UartCmd_Poll(arm);

        uint16_t motors[4] = { ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US };
        FlightCmd cmd = { .throttle_us = (float)ESC_MIN_US };
        FlightCtrlDebug dbg = { 0 };

        if (arm == ARM_STATE_ARMED) {
            cmd = FlightInput_Get(dt);
            FlightCtrl_Update(&s, &cmd, dt, motors, &dbg);
        } else {
            /* Not armed: clean PID state and idle throttle for the next arm. */
            FlightCtrl_Idle(&s);
            FlightInput_Reset();
        }

        ESC_SetAllMotors(motors[0], motors[1], motors[2], motors[3]);

        /* Telemetry — one CSV line every 2 ticks (~50 Hz).
         * 100 Hz overruns the 16-deep log queue during motion bursts (~33%
         * loss observed). 50 Hz drains comfortably under UART 115200.
         * Format: t_ms,roll,pitch,gx,gy,gz,m1,m2,m3,m4,arm,thr,mode,rsp,psp
         * arm: 0 = disarmed, 1 = armed, 2 = arming
         * mode: 0 = rate, 1 = angle
         * rsp/psp: roll/pitch rate setpoints from the angle loop, deg/s */
        if ((++log_div & 0x1) == 0u) {
            Log_Printf("%lu,%.1f,%.1f,%.1f,%.1f,%.1f,%u,%u,%u,%u,%u,%u,%u,%.1f,%.1f\r\n",
                       (unsigned long)now,
                       s.roll,   s.pitch,
                       s.gyro_x, s.gyro_y, s.gyro_z,
                       motors[0], motors[1], motors[2], motors[3],
                       (unsigned)arm,
                       (unsigned)cmd.throttle_us,
                       (unsigned)FlightCtrl_GetMode(),
                       dbg.roll_rate_sp, dbg.pitch_rate_sp);
        }
    }
}

/* USER CODE END Application */

