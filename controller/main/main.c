/*
 * FalconDrone controller — entry point and radio task.
 *
 * radio_task, every RC_PERIOD_MS (50 Hz):
 *   1. read slider, stick and button (inputs.c)
 *   2. send one RcPacket (nRF24 PTX, auto-ACK with retries)
 *   3. wait for the result; the ACK carries the drone's RcAck status
 *
 * Arming happens on the drone: hold the stick button 2 s with the slider at
 * zero; press it again to disarm. The drone disarms by itself if packets
 * stop for RC_LINK_TIMEOUT_MS.
 *
 * Status goes to the serial monitor twice a second.
 */

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "board.h"
#include "inputs.h"
#include "nrf24.h"
#include "nrf24_port_esp32.h"
#include "rc_protocol.h"

#define TX_WAIT_MAX_MS   10      /* worst case with 5 retries is ~4 ms        */
#define ACK_STALE_MS     250     /* no ACK payload this long → drone unknown  */
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

static void radio_init(Nrf24 *radio)
{
    Nrf24Port port;
    ESP_ERROR_CHECK(nrf24_port_esp32_init(&port));

    const Nrf24Config cfg = {
        .role           = NRF24_ROLE_PTX,
        .channel        = RC_CHANNEL,
        .rate           = NRF24_RATE_1MBPS,
        .power          = NRF24_PA_LOW,   /* bench: modules sit close together */
        .address        = RC_ADDRESS,
        .retries        = RC_RETRIES,
        .retry_delay_us = RC_RETRY_DELAY_US,
    };

    vTaskDelay(pdMS_TO_TICKS(100));       /* nRF24 power-on reset */
    while (nrf24_init(radio, &port, &cfg) != NRF24_OK) {
        ESP_LOGE(TAG, "nRF24 not found — check wiring and the 3V3 capacitor; retrying");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "radio ready, channel %u", (unsigned)RC_CHANNEL);
}

static void radio_task(void *arg)
{
    (void)arg;
    Nrf24 radio;
    radio_init(&radio);

    uint8_t  seq = 0;
    RcAck    drone = { 0 };
    TickType_t last_ack = 0;
    bool     have_ack = false;

    /* Link quality, counted per status period. */
    unsigned sent = 0, acked = 0;
    TickType_t last_status = xTaskGetTickCount();

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
        nrf24_send(&radio, &pkt, sizeof(pkt));
        sent++;

        Nrf24TxResult res = NRF24_TX_PENDING;
        uint8_t ack[NRF24_MAX_PAYLOAD];
        uint8_t ack_len = 0;
        for (int ms = 0; ms < TX_WAIT_MAX_MS && res == NRF24_TX_PENDING; ms++) {
            vTaskDelay(1);
            res = nrf24_poll_tx(&radio, ack, &ack_len);
        }

        const TickType_t now = xTaskGetTickCount();
        if (res == NRF24_TX_OK) {
            acked++;
            if (ack_len == sizeof(RcAck)) {
                memcpy(&drone, ack, sizeof(drone));
                last_ack = now;
                have_ack = true;
            }
        }

        if (now - last_status >= pdMS_TO_TICKS(STATUS_PERIOD_MS)) {
            const unsigned link = sent ? acked * 100u / sent : 0u;
            const bool drone_fresh = have_ack && (now - last_ack) <= pdMS_TO_TICKS(ACK_STALE_MS);
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
    xTaskCreate(radio_task, "radio", 4096, NULL, 10, NULL);
}
