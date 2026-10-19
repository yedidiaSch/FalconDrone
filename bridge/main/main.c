/*
 * FalconDrone radio bridge — sits on the drone between the air and the
 * STM32:
 *
 *   controller ──ESP-NOW── bridge ──UART── STM32
 *
 * It only forwards rc_frame.h frames, never makes up data of its own (apart
 * from HELLO, which carries none). If the bridge stops, frames stop, and the
 * STM32's link timeout disarms — the same failsafe as a dead radio.
 *
 *   air → UART  RcPacket frames from the controller we are locked to
 *   UART → air  RcAck frames from the STM32, to that controller
 *
 * Pairing: the first controller whose packet arrives is the one we serve
 * until reboot. Packets it sends to the broadcast address (it does not know
 * us yet) get a HELLO back, so it can switch to unicast.
 */

#include <string.h>
#include "board.h"
#include "espnow_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "rc_frame.h"
#include "rc_protocol.h"

#define UART_RX_BUF     512
#define AIR_QUEUE_LEN   8
#define STATUS_MS       2000

static const char *TAG = "bridge";

typedef struct {
    uint8_t src[ESP_NOW_ETH_ALEN];
    bool    broadcast;            /* sent to FF:FF:..., the controller is searching */
    uint8_t len;
    uint8_t frame[RC_FRAME_MAX];
} AirFrame;

static QueueHandle_t s_air_q;

/* Only touched by air_task once set. */
static uint8_t s_ctrl[ESP_NOW_ETH_ALEN];
static volatile bool s_locked;

/* Counters for the status line. */
static volatile uint32_t s_air_in, s_air_dropped, s_ack_out, s_ack_fail, s_hello_out;

/* Wi-Fi task context: validate, then hand off. */
static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    uint8_t type, plen;
    const uint8_t *payload;
    if (len <= 0 || !rc_frame_decode(data, (size_t)len, &type, &payload, &plen)
        || type != RC_FRAME_PACKET || plen != sizeof(RcPacket)) {
        return;
    }

    AirFrame f = { .len = (uint8_t)len };
    memcpy(f.src, info->src_addr, ESP_NOW_ETH_ALEN);
    f.broadcast = memcmp(info->des_addr, ESPNOW_BROADCAST, ESP_NOW_ETH_ALEN) == 0;
    memcpy(f.frame, data, (size_t)len);
    if (xQueueSend(s_air_q, &f, 0) != pdTRUE) {
        s_air_dropped++;
    }
}

static void on_send(const esp_now_send_info_t *info, esp_now_send_status_t status)
{
    (void)info;
    if (status != ESP_NOW_SEND_SUCCESS) {
        s_ack_fail++;
    }
}

static void send_hello(void)
{
    uint8_t frame[RC_FRAME_MAX];
    const size_t n = rc_frame_encode(RC_FRAME_HELLO, NULL, 0, frame, sizeof(frame));
    if (esp_now_send(s_ctrl, frame, n) == ESP_OK) {
        s_hello_out++;
    }
}

/* air → UART */
static void air_task(void *arg)
{
    (void)arg;
    AirFrame f;
    for (;;) {
        xQueueReceive(s_air_q, &f, portMAX_DELAY);

        if (!s_locked) {
            if (espnow_wifi_add_peer(f.src) != ESP_OK) {
                continue;
            }
            memcpy(s_ctrl, f.src, ESP_NOW_ETH_ALEN);
            s_locked = true;
            ESP_LOGI(TAG, "serving controller " MACSTR, MAC2STR(s_ctrl));
        }
        if (memcmp(f.src, s_ctrl, ESP_NOW_ETH_ALEN) != 0) {
            continue;                                  /* someone else's */
        }
        if (f.broadcast) {
            send_hello();
        }
        uart_write_bytes(BRIDGE_UART, f.frame, f.len);
        s_air_in++;
    }
}

/* UART → air */
static void uart_task(void *arg)
{
    (void)arg;
    RcFrameParser parser;
    rc_frame_parser_init(&parser);

    uint8_t buf[64];
    for (;;) {
        const int n = uart_read_bytes(BRIDGE_UART, buf, sizeof(buf), pdMS_TO_TICKS(5));
        for (int i = 0; i < n; i++) {
            if (!rc_frame_parse_byte(&parser, buf[i])
                || parser.type != RC_FRAME_ACK || parser.len != sizeof(RcAck)) {
                continue;
            }
            if (!s_locked) {
                continue;                              /* nobody to tell yet */
            }
            uint8_t frame[RC_FRAME_MAX];
            const size_t len = rc_frame_encode(RC_FRAME_ACK, parser.payload, parser.len,
                                               frame, sizeof(frame));
            if (esp_now_send(s_ctrl, frame, len) == ESP_OK) {
                s_ack_out++;
            } else {
                s_ack_fail++;
            }
        }

        static TickType_t last_status;
        const TickType_t now = xTaskGetTickCount();
        if (now - last_status >= pdMS_TO_TICKS(STATUS_MS)) {
            last_status = now;
            if (!s_locked) {
                ESP_LOGW(TAG, "waiting for the controller (Wi-Fi channel %u)", (unsigned)RC_WIFI_CHANNEL);
            } else {
                ESP_LOGI(TAG, "air→stm %lu  stm→air %lu  (fail %lu, dropped %lu, uart crc err %lu)%s",
                         (unsigned long)s_air_in, (unsigned long)s_ack_out,
                         (unsigned long)s_ack_fail, (unsigned long)s_air_dropped,
                         (unsigned long)parser.errors,
                         s_air_in > 0 && s_ack_out == 0 ? " — STM32 is not answering" : "");
            }
            s_air_in = s_ack_out = s_ack_fail = s_air_dropped = 0;
        }
    }
}

static void uart_start(void)
{
    const uart_config_t cfg = {
        .baud_rate  = RC_BRIDGE_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(BRIDGE_UART, UART_RX_BUF, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(BRIDGE_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(BRIDGE_UART, PIN_BRIDGE_TX, PIN_BRIDGE_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    /* Idle-high when the STM32 is unplugged, instead of floating noise. */
    gpio_pullup_en(PIN_BRIDGE_RX);
}

void app_main(void)
{
    s_air_q = xQueueCreate(AIR_QUEUE_LEN, sizeof(AirFrame));
    configASSERT(s_air_q);
    uart_start();
    ESP_ERROR_CHECK(espnow_wifi_start(on_send, on_recv));

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "FalconDrone bridge up — MAC " MACSTR ", Wi-Fi channel %u, UART%d TX=%d RX=%d @ %u",
             MAC2STR(mac), (unsigned)RC_WIFI_CHANNEL, BRIDGE_UART,
             PIN_BRIDGE_TX, PIN_BRIDGE_RX, (unsigned)RC_BRIDGE_BAUD);

    xTaskCreate(air_task,  "air",  3072, NULL, 12, NULL);
    xTaskCreate(uart_task, "uart", 3072, NULL, 11, NULL);
}
