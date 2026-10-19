#include "ui.h"
#include "oled.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ui";

static UiState     s_state;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

void ui_publish(const UiState *s)
{
    taskENTER_CRITICAL(&s_lock);
    s_state = *s;
    taskEXIT_CRITICAL(&s_lock);
}

static void ui_task(void *arg)
{
    (void)arg;
    static Gfx g;                     /* 1 KB — keep it off the task stack */
    unsigned errors = 0;
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        UiState s;
        taskENTER_CRITICAL(&s_lock);
        s = s_state;
        taskEXIT_CRITICAL(&s_lock);
        s.now_ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());

        ui_render(&g, &s);
        if (oled_flush(&g) != ESP_OK && errors++ % 50u == 0u) {
            ESP_LOGW(TAG, "display write failed (loose wire?)");
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / UI_HZ));
    }
}

void ui_start(void)
{
    if (oled_init() != ESP_OK) {
        ESP_LOGW(TAG, "running without a display");
        return;
    }
    /* Below the radio task: a slow I2C frame must never delay a packet. */
    xTaskCreate(ui_task, "ui", 3072, NULL, 3, NULL);
}
