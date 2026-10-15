#ifndef __RESET_INFO_H__
#define __RESET_INFO_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Boot diagnostics: why did the MCU reset, and is brown-out protection set.
 *
 * Call order:
 *   main(), first line          ResetInfo_Capture()
 *   main(), after clock config  ResetInfo_EnsureBrownoutLevel()
 *   any task, after scheduler   ResetInfo_Report()   (needs the logger)
 */

typedef enum {
    RESET_CAUSE_UNKNOWN = 0,
    RESET_CAUSE_POWER_ON,     /* supply came up from off             */
    RESET_CAUSE_BROWNOUT,     /* supply sagged below the BOR level   */
    RESET_CAUSE_PIN,          /* NRST: RESET button                  */
    RESET_CAUSE_SOFTWARE,     /* NVIC_SystemReset — also st-flash    */
    RESET_CAUSE_IWDG,         /* independent watchdog                */
    RESET_CAUSE_WWDG,         /* window watchdog                     */
    RESET_CAUSE_LOW_POWER,    /* illegal Stop/Standby entry          */
} ResetCause_t;

/* Latch and clear the RCC reset flags. Must run before anything else
 * can reset or clear them. */
void ResetInfo_Capture(void);

/* Program the BOR option byte to level 3 (~2.7 V) if it isn't already.
 * One-time flash write; a no-op on every later boot. Needs HAL_Init. */
void ResetInfo_EnsureBrownoutLevel(void);

/* Log the reset cause (with the watchdog post-mortem) and the BOR level. */
void ResetInfo_Report(void);

ResetCause_t ResetInfo_GetCause(void);

#ifdef __cplusplus
}
#endif

#endif /* __RESET_INFO_H__ */
