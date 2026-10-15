#include "watchdog.h"
#include "logger.h"
#include "cmsis_os2.h"
#include "stm32f4xx.h"

/*
 * IWDG timing. The IWDG runs from the LSI (~32 kHz, datasheet range
 * 17-47 kHz), so the timeout is approximate:
 *   (RELOAD + 1) * 64 / 32 kHz = 0.5 s nominal, ~0.34-0.94 s worst case.
 * Supervisor refreshes every 50 ms, so the margin is large.
 *
 * Registers are driven directly — the project has no HAL/LL IWDG driver.
 */
#define IWDG_KEY_START    0xCCCCu
#define IWDG_KEY_UNLOCK   0x5555u
#define IWDG_KEY_REFRESH  0xAAAAu
#define IWDG_PRESCALER_64 4u
#define IWDG_RELOAD       249u

/* Every monitored task must have checked in at least once by then. */
#define WDG_BOOT_GRACE_MS 10000u

#define WDG_STALL_MAGIC   0x57444721u   /* "WDG!" */

typedef struct {
    const char *name;
    uint32_t    deadline_ms;
} WdgSpec_t;

static const WdgSpec_t s_spec[WDG_COUNT] = {
    [WDG_IMU]     = { "IMUTask",     2000u },  /* > 1.05 s calibration block */
    [WDG_CONTROL] = { "ControlTask",  300u },  /* loop runs every 10-100 ms  */
};

/* Written by the monitored tasks, read by the supervisor. Word-sized, so
 * loads/stores are atomic on Cortex-M4. */
static volatile uint32_t s_last_ms[WDG_COUNT];
static volatile uint8_t  s_registered[WDG_COUNT];

static uint8_t  s_started;
static uint8_t  s_tripped;
static uint32_t s_start_ms;

static volatile uint8_t s_test_hang;

/* Survives the watchdog reset (.noinit is neither zeroed nor loaded by the
 * startup code). After power-on it holds garbage — `check` rejects that. */
typedef struct {
    uint32_t magic;
    uint32_t task;
    uint32_t silent_ms;
    uint32_t check;
} StallRecord_t;

static StallRecord_t s_stall __attribute__((section(".noinit")));

static void iwdg_start(void)
{
    /* Stop the IWDG while the core is halted by a debugger, otherwise every
     * breakpoint resets the board. */
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;

    IWDG->KR  = IWDG_KEY_START;      /* also switches the LSI on */
    IWDG->KR  = IWDG_KEY_UNLOCK;
    IWDG->PR  = IWDG_PRESCALER_64;
    IWDG->RLR = IWDG_RELOAD;
    while (IWDG->SR != 0u) {
        /* PVU/RVU: new values take a few LSI cycles to latch */
    }
    IWDG->KR  = IWDG_KEY_REFRESH;
}

static void trip(WdgTask_t task, uint32_t silent_ms)
{
    s_tripped = 1;

    s_stall.magic     = WDG_STALL_MAGIC;
    s_stall.task      = (uint32_t)task;
    s_stall.silent_ms = silent_ms;
    s_stall.check     = WDG_STALL_MAGIC ^ (uint32_t)task ^ silent_ms;

    /* May or may not reach the UART before the reset — the .noinit record
     * is the reliable copy. */
    Log_Printf("[WDG] %s silent %lu ms - watchdog reset\r\n",
               s_spec[task].name, (unsigned long)silent_ms);
}

void Watchdog_CheckIn(WdgTask_t task)
{
    if (task >= WDG_COUNT) return;
    s_last_ms[task] = osKernelGetTickCount();
    s_registered[task] = 1;
}

void Watchdog_Supervise(uint32_t now_ms)
{
    if (!s_started) {
        iwdg_start();
        s_started  = 1;
        s_start_ms = now_ms;
    }

    /* Once a stall is detected, never refresh again — let the reset happen. */
    if (s_tripped) return;

    for (int t = 0; t < WDG_COUNT; ++t) {
        if (!s_registered[t]) {
            const uint32_t since_boot = now_ms - s_start_ms;
            if (since_boot > WDG_BOOT_GRACE_MS) {
                trip((WdgTask_t)t, since_boot);
                return;
            }
            continue;
        }
        /* Signed: a higher-priority task may check in between our read of
         * now_ms and this load, making last_ms slightly newer than now_ms. */
        const int32_t silent = (int32_t)(now_ms - s_last_ms[t]);
        if (silent > (int32_t)s_spec[t].deadline_ms) {
            trip((WdgTask_t)t, (uint32_t)silent);
            return;
        }
    }

    IWDG->KR = IWDG_KEY_REFRESH;
}

bool Watchdog_TakeLastStall(const char **task_name, uint32_t *silent_ms)
{
    const bool valid =
        s_stall.magic == WDG_STALL_MAGIC &&
        s_stall.task < WDG_COUNT &&
        s_stall.check == (WDG_STALL_MAGIC ^ s_stall.task ^ s_stall.silent_ms);

    if (valid) {
        *task_name = s_spec[s_stall.task].name;
        *silent_ms = s_stall.silent_ms;
    }
    s_stall.magic = 0;
    return valid;
}

void Watchdog_RequestTestHang(void)
{
    s_test_hang = 1;
}

bool Watchdog_TestHangRequested(void)
{
    return s_test_hang != 0;
}
