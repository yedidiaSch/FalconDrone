#include "espnow_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "rc_protocol.h"
#include <stdbool.h>
#include <string.h>

const uint8_t ESPNOW_BROADCAST[ESP_NOW_ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static esp_err_t wifi_start(void)
{
    esp_err_t err = nvs_flash_init();          /* Wi-Fi keeps calibration in NVS */
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        if ((err = nvs_flash_erase()) != ESP_OK) return err;
        err = nvs_flash_init();
    }
    if (err != ESP_OK) return err;

    if ((err = esp_netif_init()) != ESP_OK) return err;
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;   /* already there */

    const wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    if ((err = esp_wifi_init(&cfg)) != ESP_OK) return err;
    if ((err = esp_wifi_set_storage(WIFI_STORAGE_RAM)) != ESP_OK) return err;
    if ((err = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK) return err;
    if ((err = esp_wifi_start()) != ESP_OK) return err;
    if ((err = esp_wifi_set_ps(WIFI_PS_NONE)) != ESP_OK) return err;   /* lowest latency */
    return esp_wifi_set_channel(RC_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
}

esp_err_t espnow_wifi_add_peer(const uint8_t mac[ESP_NOW_ETH_ALEN])
{
    if (esp_now_is_peer_exist(mac)) {
        return ESP_OK;
    }
    esp_now_peer_info_t peer = {
        .channel = RC_WIFI_CHANNEL,
        .ifidx   = WIFI_IF_STA,
        .encrypt = false,
    };
    memcpy(peer.peer_addr, mac, ESP_NOW_ETH_ALEN);
    return esp_now_add_peer(&peer);
}

esp_err_t espnow_wifi_start(esp_now_send_cb_t on_send, esp_now_recv_cb_t on_recv)
{
    static bool s_wifi_up;
    esp_err_t err;
    if (!s_wifi_up) {
        if ((err = wifi_start()) != ESP_OK) return err;
        s_wifi_up = true;
    }

    if ((err = esp_now_init()) != ESP_OK) return err;
    if ((on_send && (err = esp_now_register_send_cb(on_send)) != ESP_OK)
        || (err = esp_now_register_recv_cb(on_recv)) != ESP_OK
        || (err = espnow_wifi_add_peer(ESPNOW_BROADCAST)) != ESP_OK) {
        esp_now_deinit();
        return err;
    }
    return ESP_OK;
}
