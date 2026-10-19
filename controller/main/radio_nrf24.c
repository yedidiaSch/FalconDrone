/*
 * nRF24 transport: PTX with auto-ACK; the drone's RcAck comes back inside
 * the ACK payload.
 */
#include "radio_link.h"
#include "nrf24.h"
#include "nrf24_port_esp32.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define TX_WAIT_MAX_MS 10        /* worst case with 5 retries is ~4 ms */

static Nrf24     s_radio;
static Nrf24Port s_port;
static bool      s_port_ready;
static RcAck     s_ack;
static bool      s_have_ack;

bool radio_link_init(void)
{
    if (!s_port_ready) {
        if (nrf24_port_esp32_init(&s_port) != ESP_OK) {
            return false;
        }
        s_port_ready = true;
        vTaskDelay(pdMS_TO_TICKS(100));   /* nRF24 power-on reset */
    }

    const Nrf24Config cfg = {
        .role           = NRF24_ROLE_PTX,
        .channel        = RC_CHANNEL,
        .rate           = NRF24_RATE_1MBPS,
        .power          = NRF24_PA_LOW,   /* bench: modules sit close together */
        .address        = RC_ADDRESS,
        .retries        = RC_RETRIES,
        .retry_delay_us = RC_RETRY_DELAY_US,
    };
    return nrf24_init(&s_radio, &s_port, &cfg) == NRF24_OK;
}

bool radio_link_send(const RcPacket *pkt)
{
    nrf24_send(&s_radio, pkt, sizeof(*pkt));

    Nrf24TxResult res = NRF24_TX_PENDING;
    uint8_t ack[NRF24_MAX_PAYLOAD];
    uint8_t ack_len = 0;
    for (int ms = 0; ms < TX_WAIT_MAX_MS && res == NRF24_TX_PENDING; ms++) {
        vTaskDelay(1);
        res = nrf24_poll_tx(&s_radio, ack, &ack_len);
    }

    if (res == NRF24_TX_OK && ack_len == sizeof(RcAck)) {
        memcpy(&s_ack, ack, sizeof(s_ack));
        s_have_ack = true;
    }
    return res == NRF24_TX_OK;
}

bool radio_link_take_ack(RcAck *ack)
{
    if (!s_have_ack) {
        return false;
    }
    *ack = s_ack;
    s_have_ack = false;
    return true;
}

const char *radio_link_name(void)
{
    return "nRF24";
}
