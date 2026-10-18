/*
 * FalconDrone controller — entry point.
 *
 * Planned task layout (mirrors the drone side):
 *   input_task  100 Hz   ADC (slider, stick X/Y) + button edges
 *   radio_task   50 Hz   build RcPacket, send over nRF24, read RcAck
 *   ui_task      10 Hz   OLED: arm state, throttle, link quality
 *
 * For now this only proves the toolchain and the shared rc_protocol
 * component; the radio link test comes next.
 */

#include "esp_log.h"
#include "board.h"
#include "rc_protocol.h"

static const char *TAG = "controller";

void app_main(void)
{
    ESP_LOGI(TAG, "FalconDrone controller boot");
    ESP_LOGI(TAG, "RcPacket %u bytes, RcAck %u bytes, channel %u, %u Hz",
             (unsigned)sizeof(RcPacket), (unsigned)sizeof(RcAck),
             (unsigned)RC_CHANNEL, (unsigned)(1000u / RC_PERIOD_MS));
}
