#ifndef RC_FRAME_H
#define RC_FRAME_H

/*
 * Framing for RcPacket / RcAck on byte links — the ESP-NOW bridge and its
 * UART to the STM32. Pure C, no platform code (host-tested).
 *
 *   0xA5 0x5A | type | len | payload[len] | crc8
 *
 * The CRC (CRC-8, poly 0x07) covers type, len and payload. On a UART the
 * parser finds frame starts in a byte stream and drops corrupted frames;
 * over ESP-NOW each radio packet carries exactly one frame, so a stray
 * ESP-NOW sender cannot inject packets that merely have the right length.
 *
 * The nRF24 link does not use this: the chip does its own framing and CRC.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RC_FRAME_SYNC0        0xA5u
#define RC_FRAME_SYNC1        0x5Au
#define RC_FRAME_MAX_PAYLOAD  16u
#define RC_FRAME_OVERHEAD     5u     /* 2 sync + type + len + crc */
#define RC_FRAME_MAX          (RC_FRAME_MAX_PAYLOAD + RC_FRAME_OVERHEAD)

typedef enum {
    RC_FRAME_PACKET = 1,   /* RcPacket: controller → drone                    */
    RC_FRAME_ACK    = 2,   /* RcAck:    drone → controller                    */
    RC_FRAME_HELLO  = 3,   /* bridge → controller, no payload: "I'm here"     */
} RcFrameType;

/* Encode one frame into out. Returns its size, or 0 if len is too big or
 * out is too small. */
size_t rc_frame_encode(uint8_t type, const void *payload, uint8_t len,
                       uint8_t *out, size_t out_size);

/* Decode a buffer holding exactly one frame (one ESP-NOW packet). On success
 * *payload points into data. */
bool rc_frame_decode(const uint8_t *data, size_t size,
                     uint8_t *type, const uint8_t **payload, uint8_t *len);

/* ---- Stream parser (UART) ------------------------------------------------ */

typedef struct {
    uint8_t  state;
    uint8_t  type;
    uint8_t  len;
    uint8_t  pos;
    uint8_t  crc;
    uint8_t  payload[RC_FRAME_MAX_PAYLOAD];
    uint32_t frames;       /* good frames                                   */
    uint32_t errors;       /* bad CRC or impossible length                  */
} RcFrameParser;

void rc_frame_parser_init(RcFrameParser *p);

/* Feed one byte. Returns true when a complete, valid frame has arrived; it
 * is in p->type, p->len and p->payload until the next call. */
bool rc_frame_parse_byte(RcFrameParser *p, uint8_t byte);

uint8_t rc_crc8(uint8_t crc, const uint8_t *data, size_t len);

#endif /* RC_FRAME_H */
