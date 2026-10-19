#include "rc_radio.h"
#include "nrf24.h"
#include "nrf24_port_stm32.h"
#include <string.h>

static Nrf24 s_radio;

bool RcRadio_Init(void)
{
    static const Nrf24Config cfg = {
        .role           = NRF24_ROLE_PRX,
        .channel        = RC_CHANNEL,
        .rate           = NRF24_RATE_1MBPS,
        .power          = NRF24_PA_LOW,   /* bench: modules sit close together */
        .address        = RC_ADDRESS,
        .retries        = RC_RETRIES,
        .retry_delay_us = RC_RETRY_DELAY_US,
    };

    if (nrf24_init(&s_radio, Nrf24Port_Stm32(), &cfg) != NRF24_OK) {
        return false;
    }
    nrf24_start_listening(&s_radio);
    return true;
}

bool RcRadio_Receive(RcPacket *pkt)
{
    while (nrf24_available(&s_radio)) {
        uint8_t buf[NRF24_MAX_PAYLOAD];
        const uint8_t len = nrf24_read(&s_radio, buf);
        if (len == sizeof(RcPacket)) {   /* anything else is not ours */
            memcpy(pkt, buf, sizeof(*pkt));
            return true;
        }
    }
    return false;
}

/* Rides on the auto-ACK of the controller's next packet. */
void RcRadio_Reply(const RcAck *ack)
{
    nrf24_set_ack_payload(&s_radio, ack, sizeof(*ack));
}

const char *RcRadio_Name(void)
{
    return "nRF24";
}

const char *RcRadio_MissingHint(void)
{
    return "nRF24 not found — check wiring (PB1/PB12-15, 3V3)";
}
