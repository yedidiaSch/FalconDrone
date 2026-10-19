#ifndef RADIO_LINK_H
#define RADIO_LINK_H

/*
 * Controller radio transport, picked in menuconfig (FalconDrone → Radio
 * transport). Exactly one implementation is built:
 *
 *   ESP-NOW (default)  radio_espnow.c — to the bridge ESP32 on the drone
 *   nRF24              radio_nrf24.c  — nRF24L01+ PTX on VSPI
 *
 * Radio task only.
 */

#include <stdbool.h>
#include "rc_protocol.h"

/* Bring the radio up. false = hardware missing; the caller retries. */
bool radio_link_init(void);

/* Send one packet and wait (a few ms) for the radio-level result. true =
 * acknowledged: by the drone's nRF24, or by the bridge over ESP-NOW. */
bool radio_link_send(const RcPacket *pkt);

/* Newest drone status received since the last call. */
bool radio_link_take_ack(RcAck *ack);

const char *radio_link_name(void);

#endif /* RADIO_LINK_H */
