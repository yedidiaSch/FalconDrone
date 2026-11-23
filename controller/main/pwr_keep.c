/*
 * IP5306 keep-alive — see pwr_keep.h.
 *
 * IP5306 KEY timing (Injoinic datasheet V1.10, KEY section):
 *   press 30 ms .. 2 s          short press: boost on, show the fuel gauge
 *   two short presses < 1 s     power OFF
 *   press > 2 s                 power OFF
 * So a pulse must stay well under 2 s, and two pulses must never land within
 * 1 s of each other — that includes the user's own press that switched the
 * RC on. The first pulse therefore waits a full period after boot.
 *
 * Q2's gate has a 100k pull-down on the power board, so the button is
 * released while the ESP32 is in reset or the pin is still an input.
 */

#include "pwr_keep.h"

#include "board.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define PWR_KEEP_PERIOD_MS   10000   /* well inside the 32 s light-load timer */
#define PWR_KEEP_PULSE_MS    100     /* > 30 ms debounce, << 2 s long press   */
#define PWR_OFF_HOLD_MS      2500    /* > 2 s long press                      */

static const char *TAG = "pwr_keep";

#if CONFIG_FALCON_PWR_KEEP

static void press(uint32_t ms)
{
    gpio_set_level(PIN_PWR_KEEP, 1);
    vTaskDelay(pdMS_TO_TICKS(ms));
    gpio_set_level(PIN_PWR_KEEP, 0);
}

static void pwr_keep_task(void *arg)
{
    (void)arg;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(PWR_KEEP_PERIOD_MS));
        press(PWR_KEEP_PULSE_MS);
    }
}

void pwr_keep_start(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << PIN_PWR_KEEP,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_set_level(PIN_PWR_KEEP, 0);
    ESP_ERROR_CHECK(gpio_config(&cfg));
    /* Above the UI, below the radio task (10): a pulse is never urgent to the
     * millisecond, but must not starve behind the display. */
    xTaskCreate(pwr_keep_task, "pwr_keep", 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "IP5306 keep-alive on GPIO%d every %d ms", PIN_PWR_KEEP, PWR_KEEP_PERIOD_MS);
}

void pwr_keep_power_off(void)
{
    ESP_LOGW(TAG, "powering off");
    press(PWR_OFF_HOLD_MS);
}

#else  /* !CONFIG_FALCON_PWR_KEEP */

void pwr_keep_start(void)
{
    ESP_LOGI(TAG, "disabled (CONFIG_FALCON_PWR_KEEP off)");
}

void pwr_keep_power_off(void) {}

#endif
