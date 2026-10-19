#include "rc_frame.h"
#include <string.h>

enum { ST_SYNC0, ST_SYNC1, ST_TYPE, ST_LEN, ST_PAYLOAD, ST_CRC };

uint8_t rc_crc8(uint8_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            const unsigned shifted = (unsigned)crc << 1;
            crc = (uint8_t)((crc & 0x80u) ? (shifted ^ 0x07u) : shifted);
        }
    }
    return crc;
}

size_t rc_frame_encode(uint8_t type, const void *payload, uint8_t len,
                       uint8_t *out, size_t out_size)
{
    const size_t size = (size_t)len + RC_FRAME_OVERHEAD;
    if (len > RC_FRAME_MAX_PAYLOAD || out_size < size || (len > 0 && payload == NULL)) {
        return 0;
    }
    out[0] = RC_FRAME_SYNC0;
    out[1] = RC_FRAME_SYNC1;
    out[2] = type;
    out[3] = len;
    if (len > 0) {
        memcpy(&out[4], payload, len);
    }
    out[4 + len] = rc_crc8(0, &out[2], (size_t)len + 2u);
    return size;
}

bool rc_frame_decode(const uint8_t *data, size_t size,
                     uint8_t *type, const uint8_t **payload, uint8_t *len)
{
    if (size < RC_FRAME_OVERHEAD || data[0] != RC_FRAME_SYNC0 || data[1] != RC_FRAME_SYNC1) {
        return false;
    }
    const uint8_t n = data[3];
    if (n > RC_FRAME_MAX_PAYLOAD || size != (size_t)n + RC_FRAME_OVERHEAD) {
        return false;
    }
    if (rc_crc8(0, &data[2], (size_t)n + 2u) != data[4 + n]) {
        return false;
    }
    *type = data[2];
    *payload = &data[4];
    *len = n;
    return true;
}

void rc_frame_parser_init(RcFrameParser *p)
{
    memset(p, 0, sizeof(*p));
    p->state = ST_SYNC0;
}

bool rc_frame_parse_byte(RcFrameParser *p, uint8_t byte)
{
    switch (p->state) {
    case ST_SYNC0:
        if (byte == RC_FRAME_SYNC0) p->state = ST_SYNC1;
        return false;

    case ST_SYNC1:
        /* A5 A5 5A: the second A5 may be the real start. */
        p->state = byte == RC_FRAME_SYNC1 ? ST_TYPE : (byte == RC_FRAME_SYNC0 ? ST_SYNC1 : ST_SYNC0);
        return false;

    case ST_TYPE:
        p->type = byte;
        p->crc  = rc_crc8(0, &byte, 1);
        p->state = ST_LEN;
        return false;

    case ST_LEN:
        if (byte > RC_FRAME_MAX_PAYLOAD) {
            p->errors++;
            p->state = ST_SYNC0;
            return false;
        }
        p->len = byte;
        p->pos = 0;
        p->crc = rc_crc8(p->crc, &byte, 1);
        p->state = byte > 0 ? ST_PAYLOAD : ST_CRC;
        return false;

    case ST_PAYLOAD:
        p->payload[p->pos++] = byte;
        p->crc = rc_crc8(p->crc, &byte, 1);
        if (p->pos == p->len) p->state = ST_CRC;
        return false;

    case ST_CRC:
    default:
        p->state = ST_SYNC0;
        if (byte != p->crc) {
            p->errors++;
            return false;
        }
        p->frames++;
        return true;
    }
}
