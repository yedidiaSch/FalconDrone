#ifndef ESPNOW_WIFI_H
#define ESPNOW_WIFI_H

/*
 * Wi-Fi + ESP-NOW bring-up shared by the controller and the drone's bridge,
 * so both always sit on the same channel with the same settings:
 * station mode, no access point, power save off, channel RC_WIFI_CHANNEL,
 * broadcast peer added.
 */

#include "esp_err.h"
#include "esp_now.h"

/* Start Wi-Fi (once) and ESP-NOW, register the callbacks. */
esp_err_t espnow_wifi_start(esp_now_send_cb_t on_send, esp_now_recv_cb_t on_recv);

/* Add a unicast peer on RC_WIFI_CHANNEL (no-op if it already exists). */
esp_err_t espnow_wifi_add_peer(const uint8_t mac[ESP_NOW_ETH_ALEN]);

extern const uint8_t ESPNOW_BROADCAST[ESP_NOW_ETH_ALEN];

#endif /* ESPNOW_WIFI_H */
