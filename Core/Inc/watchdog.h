#ifndef __WATCHDOG_H__
#define __WATCHDOG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/*
 * Supervised independent watchdog (IWDG).
 *
 * Flight-critical tasks check in every loop. The supervisor (DefaultTask,
 * every WATCHDOG_SUPERVISE_PERIOD_MS) refreshes the IWDG only while every
 * monitored task has checked in within its deadline. A silent task — or a
 * higher-priority task starving the supervisor — stops the refresh and the
 * IWDG resets the MCU after ~0.5 s.
 *
 * LogTask is deliberately not monitored: a stuck logger must never reboot
 * the flight controller.
 */

#define WATCHDOG_SUPERVISE_PERIOD_MS 50u

typedef enum {
    WDG_IMU = 0,    /* IMUTask — deadline covers the ~1 s gyro calibration */
    WDG_CONTROL,    /* ControlTask — drives the motors                     */
    WDG_COUNT
} WdgTask_t;

/* Called by a monitored task once per loop. A task becomes monitored on its
 * first check-in. Task context only. */
void Watchdog_CheckIn(WdgTask_t task);

/* Called by DefaultTask every WATCHDOG_SUPERVISE_PERIOD_MS. Starts the IWDG
 * on the first call. */
void Watchdog_Supervise(uint32_t now_ms);

/*
 * Post-mortem of the last watchdog trip, kept in .noinit RAM across the reset.
 * Returns false if the supervisor never recorded one (e.g. it was starved).
 * Consumes the record.
 */
bool Watchdog_TakeLastStall(const char **task_name, uint32_t *silent_ms);

/* Bench test: ask ControlTask to hang on purpose (only honoured while
 * DISARMED). ISR-safe. */
void Watchdog_RequestTestHang(void);
bool Watchdog_TestHangRequested(void);

#ifdef __cplusplus
}
#endif

#endif /* __WATCHDOG_H__ */
