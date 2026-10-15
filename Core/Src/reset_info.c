#include "reset_info.h"
#include "watchdog.h"
#include "logger.h"
#include "stm32f4xx_hal.h"

typedef enum {
    BOR_STATUS_UNCHECKED = 0,
    BOR_STATUS_OK,          /* already at level 3                  */
    BOR_STATUS_PROGRAMMED,  /* written this boot                   */
    BOR_STATUS_FAILED,      /* option-byte write failed            */
} BorStatus_t;

static ResetCause_t s_cause = RESET_CAUSE_UNKNOWN;
static BorStatus_t  s_bor   = BOR_STATUS_UNCHECKED;

void ResetInfo_Capture(void)
{
    const uint32_t csr = RCC->CSR;

    /* Priority matters: PINRSTF is set on *every* reset (the internal reset
     * drives the NRST pin), and PORRSTF comes together with BORRSTF. */
    if      (csr & RCC_CSR_IWDGRSTF) s_cause = RESET_CAUSE_IWDG;
    else if (csr & RCC_CSR_WWDGRSTF) s_cause = RESET_CAUSE_WWDG;
    else if (csr & RCC_CSR_LPWRRSTF) s_cause = RESET_CAUSE_LOW_POWER;
    else if (csr & RCC_CSR_SFTRSTF)  s_cause = RESET_CAUSE_SOFTWARE;
    else if (csr & RCC_CSR_PORRSTF)  s_cause = RESET_CAUSE_POWER_ON;
    else if (csr & RCC_CSR_BORRSTF)  s_cause = RESET_CAUSE_BROWNOUT;
    else if (csr & RCC_CSR_PINRSTF)  s_cause = RESET_CAUSE_PIN;
    else                             s_cause = RESET_CAUSE_UNKNOWN;

    /* Clear so the next reset reports only its own cause. */
    RCC->CSR |= RCC_CSR_RMVF;
}

void ResetInfo_EnsureBrownoutLevel(void)
{
    FLASH_OBProgramInitTypeDef ob = {0};
    HAL_FLASHEx_OBGetConfig(&ob);

    if (ob.BORLevel == OB_BOR_LEVEL3) {
        s_bor = BOR_STATUS_OK;
        return;
    }

    /* Factory default is BOR off: the core keeps running down to ~1.7 V,
     * where flash reads and I2C get unreliable. Level 3 holds the MCU in
     * reset below ~2.7 V instead — a clean restart if the ESC BEC sags.
     * Only the BOR_LEV bits are touched; RDP and user bits are untouched. */
    ob.OptionType = OPTIONBYTE_BOR;
    ob.BORLevel   = OB_BOR_LEVEL3;

    HAL_FLASH_Unlock();
    HAL_FLASH_OB_Unlock();
    const bool ok = (HAL_FLASHEx_OBProgram(&ob) == HAL_OK) &&
                    (HAL_FLASH_OB_Launch() == HAL_OK);
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();

    s_bor = ok ? BOR_STATUS_PROGRAMMED : BOR_STATUS_FAILED;
}

static const char *cause_name(ResetCause_t c)
{
    switch (c) {
    case RESET_CAUSE_POWER_ON:  return "POWER-ON";
    case RESET_CAUSE_BROWNOUT:  return "BROWNOUT (supply sagged)";
    case RESET_CAUSE_PIN:       return "RESET PIN";
    case RESET_CAUSE_SOFTWARE:  return "SOFTWARE (flash/debugger)";
    case RESET_CAUSE_IWDG:      return "WATCHDOG";
    case RESET_CAUSE_WWDG:      return "WINDOW WATCHDOG";
    case RESET_CAUSE_LOW_POWER: return "LOW-POWER";
    default:                    return "UNKNOWN";
    }
}

void ResetInfo_Report(void)
{
    const char *task;
    uint32_t silent_ms;
    const bool have_stall = Watchdog_TakeLastStall(&task, &silent_ms);

    if (s_cause == RESET_CAUSE_IWDG) {
        if (have_stall) {
            Log_Printf("[SYS] Reset cause: WATCHDOG - %s silent %lu ms\r\n",
                       task, (unsigned long)silent_ms);
        } else {
            /* The supervisor never got to record anything: something at
             * higher priority than DefaultTask hogged the CPU. */
            Log_Print("[SYS] Reset cause: WATCHDOG - supervisor starved "
                      "(a higher-priority task hogged the CPU)\r\n");
        }
    } else {
        Log_Printf("[SYS] Reset cause: %s\r\n", cause_name(s_cause));
    }

    switch (s_bor) {
    case BOR_STATUS_OK:
        Log_Print("[SYS] Brown-out reset: level 3 (~2.7 V)\r\n");
        break;
    case BOR_STATUS_PROGRAMMED:
        Log_Print("[SYS] Brown-out reset: level 3 (~2.7 V) - programmed this boot\r\n");
        break;
    case BOR_STATUS_FAILED:
        Log_Print("[SYS] Brown-out reset: FAILED to program option byte\r\n");
        break;
    default:
        break;
    }
}

ResetCause_t ResetInfo_GetCause(void)
{
    return s_cause;
}
