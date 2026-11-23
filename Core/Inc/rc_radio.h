#ifndef __RC_RADIO_H__
#define __RC_RADIO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "rc_protocol.h"

/*
 * Radio transport under rc_service.c. One implementation is linked, picked
 * by FALCON_BOARD (or the FALCON_RC_RADIO override) in CMakeLists.txt:
 *
 *   NRF24   (CARRIER)     rc_radio_nrf24.c  — nRF24L01+ PRX on SPI2
 *   BRIDGE  (BREADBOARD)  rc_radio_bridge.c — ESP32 ESP-NOW bridge on USART1
 *                         (PA9 TX, PA10 RX), rc_frame.h frames
 *
 * ControlTask only — no locking.
 */

/* Bring the transport up. false = hardware did not answer; rc_service
 * retries every 2 s. */
bool RcRadio_Init(void);

/* Pop one received RcPacket. Call until it returns false. */
bool RcRadio_Receive(RcPacket *pkt);

/* Status for the controller, called after packets arrived. */
void RcRadio_Reply(const RcAck *ack);

const char *RcRadio_Name(void);         /* "bridge" / "nRF24" for logs       */
const char *RcRadio_MissingHint(void);  /* logged once when Init fails       */

#ifdef __cplusplus
}
#endif

#endif /* __RC_RADIO_H__ */
