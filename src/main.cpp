#include "FreeRTOS.h"
#include "task.h"

extern "C" void SystemClock_Config(void);
extern "C" void MX_GPIO_Init(void);

void LedTask(void *parameter)
{
    (void) parameter;
    while (1)
    {
        // TODO: toggle board LED here
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int main()
{
    SystemClock_Config();
    MX_GPIO_Init();

    xTaskCreate(LedTask, "LED", configMINIMAL_STACK_SIZE, nullptr, tskIDLE_PRIORITY + 1, nullptr);
    vTaskStartScheduler();

    while (1) {}
}
