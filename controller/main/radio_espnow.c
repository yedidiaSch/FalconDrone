/*
 * ESP-NOW transport: to the bridge ESP32 on the drone, which passes frames
 * to and from the STM32 over UART.
 *
 * Pairing is automatic and needs no MAC addresses in the code:
 *   1. Until paired, packets go to the broadcast address.
 *   2. The bridge answers a broadcast packet with a HELLO frame; any valid
 *      frame from a bridge pairs us with its MAC.
 *   3. From then on packets are unicast, so ESP-NOW acknowledges and
 *      retries them at the MAC level — that is the tx % on the screen.
 *      Frames from any other MAC are ignored until reboot.
 *
 * Every packet is an rc_frame.h frame (sync + CRC), so other ESP-NOW
 * traffic on the channel is dropped.
 */
#include "radio_link.h"
#include "rc_frame.h"
#include "espnow_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

#define SEND_WAIT_MS 10          /* MAC-level ACK + retries take ~1-3 ms */

static const char *TAG = "espnow";

static SemaphoreHandle_t s_send_done;
static volatile bool     s_send_ok;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static uint8_t  s_peer[ESP_NOW_ETH_ALEN];     /* bridge we are paired with   */
static bool     s_paired;                     /* peer added, unicasting      */
static uint8_t  s_candidate[ESP_NOW_ETH_ALEN];/* bridge seen, not yet added  */
static bool     s_have_candidate;
static RcAck    s_ack;
static bool     s_have_ack;

static void on_send(const esp_now_send_info_t *info, esp_now_send_status_t status)
{
    (void)info;
    s_send_ok = status == ESP_NOW_SEND_SUCCESS;
    xSemaphoreGive(s_send_done);
}

/* Wi-Fi task context: keep it short. */
static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    uint8_t type, plen;
    const uint8_t *payload;
    if (len <= 0 || !rc_frame_decode(data, (size_t)len, &type, &payload, &plen)) {
        return;
    }

    portENTER_CRITICAL(&s_lock);
    const bool from_peer = s_paired && memcmp(info->src_addr, s_peer, ESP_NOW_ETH_ALEN) == 0;
    if (!s_paired && !s_have_candidate && (type == RC_FRAME_HELLO || type == RC_FRAME_ACK)) {
        memcpy(s_candidate, info->src_addr, ESP_NOW_ETH_ALEN);
        s_have_candidate = true;
    }
    if (from_peer && type == RC_FRAME_ACK && plen == sizeof(RcAck)) {
        memcpy(&s_ack, payload, sizeof(s_ack));
        s_have_ack = true;
    }
    portEXIT_CRITICAL(&s_lock);
}

bool radio_link_init(void)
{
    if (s_send_done == NULL && (s_send_done = xSemaphoreCreateBinary()) == NULL) {
        return false;
    }
    const esp_err_t err = espnow_wifi_start(on_send, on_recv);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi / ESP-NOW start failed: %s", esp_err_to_name(err));
        return false;
    }

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "ESP-NOW up on Wi-Fi channel %u, my MAC " MACSTR " — looking for the bridge",
             (unsigned)RC_WIFI_CHANNEL, MAC2STR(mac));
    return true;
}

/* Pair with a bridge that answered. Radio task context. */
static void maybe_pair(void)
{
    uint8_t mac[ESP_NOW_ETH_ALEN];
    portENTER_CRITICAL(&s_lock);
    const bool have = s_have_candidate && !s_paired;
    memcpy(mac, s_candidate, sizeof(mac));
    portEXIT_CRITICAL(&s_lock);
    if (!have) {
        return;
    }

    if (espnow_wifi_add_peer(mac) == ESP_OK) {
        portENTER_CRITICAL(&s_lock);
        memcpy(s_peer, mac, sizeof(mac));
        s_paired = true;
        portEXIT_CRITICAL(&s_lock);
        ESP_LOGI(TAG, "paired with bridge " MACSTR, MAC2STR(mac));
    } else {
        portENTER_CRITICAL(&s_lock);
        s_have_candidate = false;          /* try the next one that answers */
        portEXIT_CRITICAL(&s_lock);
    }
}

bool radio_link_send(const RcPacket *pkt)
{
    maybe_pair();

    uint8_t frame[RC_FRAME_MAX];
    const size_t n = rc_frame_encode(RC_FRAME_PACKET, pkt, sizeof(*pkt), frame, sizeof(frame));

    const uint8_t *dest = s_paired ? s_peer : ESPNOW_BROADCAST;   /* s_peer is fixed once paired */
    xSemaphoreTake(s_send_done, 0);                        /* drop a stale result */
    if (esp_now_send(dest, frame, n) != ESP_OK) {
        return false;
    }
    if (xSemaphoreTake(s_send_done, pdMS_TO_TICKS(SEND_WAIT_MS)) != pdTRUE) {
        return false;
    }
    /* Broadcasts are never acknowledged — "success" there means nothing. */
    return s_paired && s_send_ok;
}

bool radio_link_take_ack(RcAck *ack)
{
    portENTER_CRITICAL(&s_lock);
    const bool have = s_have_ack;
    if (have) {
        *ack = s_ack;
        s_have_ack = false;
    }
    portEXIT_CRITICAL(&s_lock);
    return have;
}

const char *radio_link_name(void)
{
    return "ESP-NOW";
}
