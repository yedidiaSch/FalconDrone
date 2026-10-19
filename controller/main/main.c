/*
 * FalconDrone controller — entry point and radio task.
 *
 * radio_task, every RC_PERIOD_MS (50 Hz):
 *   1. read slider, stick and button (inputs.c)
 *   2. send one RcPacket over the radio transport (radio_link.h: ESP-NOW
 *      to the drone's bridge, or nRF24) and wait for the radio-level ACK
 *   3. pick up the drone's newest RcAck status
 *
 * Arming happens on the drone: hold the stick button 2 s with the slider at
 * zero; press it again to disarm. The drone disarms by itself if packets
 * stop for RC_LINK_TIMEOUT_MS.
 *
 * Status goes to the OLED (ui.c, 10 Hz) and to the serial monitor twice a
 * second.
 */

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "board.h"
#include "inputs.h"
#include "radio_link.h"
#include "rc_protocol.h"
#include "ui.h"

#define ACK_STALE_MS     250     /* no RcAck this long → drone unknown        */
#define STATUS_PERIOD_MS 500

static const char *TAG = "controller";

static const char *arm_name(uint8_t s)
{
    switch (s) {
    case 0:  return "DISARMED";
    case 1:  return "ARMED";
    case 2:  return "ARMING";
    default: return "?";
    }
}

static void radio_init(void)
{
    while (!radio_link_init()) {
        Inputs in;
        inputs_read(&in);
        const UiState ui = { .radio_ok = false, .throttle = in.throttle,
                             .roll = in.roll, .pitch = in.pitch, .button = in.button };
        ui_publish(&ui);
        ESP_LOGE(TAG, "%s radio not ready — check wiring; retrying "
                      "(thr=%u roll=%+d pitch=%+d btn=%d raw thr=%d x=%d y=%d)",
                 radio_link_name(), (unsigned)in.throttle, in.roll, in.pitch, in.button,
                 in.raw_throttle, in.raw_x, in.raw_y);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "radio ready (%s)", radio_link_name());
}

static void radio_task(void *arg)
{
    (void)arg;
    radio_init();

    uint8_t  seq = 0;
    RcAck    drone = { 0 };
    TickType_t last_ack = 0;
    bool     have_ack = false;

    /* Link quality, counted per status period. */
    unsigned sent = 0, acked = 0;
    uint8_t  tx_pct = 0;
    TickType_t last_status = xTaskGetTickCount();
    TickType_t button_down = 0;

    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(RC_PERIOD_MS));

        Inputs in;
        inputs_read(&in);

        const RcPacket pkt = {
            .seq      = seq++,
            .flags    = in.button ? RC_FLAG_ARM_REQ : 0,
            .throttle = in.throttle,
            .roll     = in.roll,
            .pitch    = in.pitch,
            .yaw      = 0,
        };
        const bool delivered = radio_link_send(&pkt);
        sent++;

        const TickType_t now = xTaskGetTickCount();
        if (delivered) {
            acked++;
        }
        if (radio_link_take_ack(&drone)) {
            last_ack = now;
            have_ack = true;
        }

        const bool drone_fresh = have_ack && (now - last_ack) <= pdMS_TO_TICKS(ACK_STALE_MS);
        if (in.button && button_down == 0) {
            button_down = now ? now : 1;     /* 0 means "not held" */
        } else if (!in.button) {
            button_down = 0;
        }
        const UiState ui = {
            .radio_ok          = true,
            .drone_link        = drone_fresh,
            .arm_state         = drone.arm_state,
            .mode              = drone.mode,
            .drone_throttle_us = drone.throttle_us,
            .tx_pct            = tx_pct,
            .rx_pct            = drone.link_pct,
            .throttle          = in.throttle,
            .roll              = in.roll,
            .pitch             = in.pitch,
            .button            = in.button,
            .button_held_ms    = button_down ? (uint32_t)pdTICKS_TO_MS(now - button_down) : 0u,
        };
        ui_publish(&ui);

        if (now - last_status >= pdMS_TO_TICKS(STATUS_PERIOD_MS)) {
            const unsigned link = sent ? acked * 100u / sent : 0u;
            tx_pct = (uint8_t)link;
            if (drone_fresh) {
                ESP_LOGI(TAG, "thr=%4u roll=%+5d pitch=%+5d btn=%d | tx %3u%% | drone %-8s thr=%uus rx=%u%%",
                         in.throttle, in.roll, in.pitch, in.button, link,
                         arm_name(drone.arm_state), drone.throttle_us, drone.link_pct);
            } else {
                ESP_LOGW(TAG, "thr=%4u roll=%+5d pitch=%+5d btn=%d | tx %3u%% | no reply from drone (raw thr=%d x=%d y=%d)",
                         in.throttle, in.roll, in.pitch, in.button, link,
                         in.raw_throttle, in.raw_x, in.raw_y);
            }
            sent = acked = 0;
            last_status = now;
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "FalconDrone controller boot — keep the stick centered");
    ESP_ERROR_CHECK(inputs_init());
    ui_start();
    xTaskCreate(radio_task, "radio", 4096, NULL, 10, NULL);
}
