/*
 * Host tests for the RcPacket/RcAck framing (bridge UART and ESP-NOW).
 *
 *   make -C shared/rc_protocol/test
 */
#include "rc_frame.h"
#include "rc_protocol.h"
#include <stdio.h>
#include <string.h>

static int s_checks, s_failed;

#define CHECK(cond) do { \
    s_checks++; \
    if (!(cond)) { s_failed++; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
} while (0)

static const RcPacket PKT = { .seq = 7, .flags = RC_FLAG_ARM | RC_FLAG_ANGLE, .throttle = 512,
                              .roll = -300, .pitch = 1000, .yaw = 0 };

/* Feed a buffer; returns how many frames completed. */
static int feed(RcFrameParser *p, const uint8_t *d, size_t n)
{
    int got = 0;
    for (size_t i = 0; i < n; i++) {
        if (rc_frame_parse_byte(p, d[i])) got++;
    }
    return got;
}

static void test_roundtrip(void)
{
    uint8_t buf[RC_FRAME_MAX];
    const size_t n = rc_frame_encode(RC_FRAME_PACKET, &PKT, sizeof(PKT), buf, sizeof(buf));
    CHECK(n == sizeof(PKT) + RC_FRAME_OVERHEAD);
    CHECK(buf[0] == RC_FRAME_SYNC0 && buf[1] == RC_FRAME_SYNC1);

    uint8_t type = 0, len = 0;
    const uint8_t *pl = NULL;
    CHECK(rc_frame_decode(buf, n, &type, &pl, &len));
    CHECK(type == RC_FRAME_PACKET && len == sizeof(PKT));
    CHECK(pl != NULL && memcmp(pl, &PKT, sizeof(PKT)) == 0);

    RcFrameParser p;
    rc_frame_parser_init(&p);
    CHECK(feed(&p, buf, n) == 1);
    CHECK(p.type == RC_FRAME_PACKET && p.len == sizeof(PKT));
    CHECK(memcmp(p.payload, &PKT, sizeof(PKT)) == 0);
    CHECK(p.frames == 1 && p.errors == 0);
}

static void test_empty_payload(void)
{
    uint8_t buf[RC_FRAME_MAX];
    const size_t n = rc_frame_encode(RC_FRAME_HELLO, NULL, 0, buf, sizeof(buf));
    CHECK(n == RC_FRAME_OVERHEAD);
    uint8_t type = 0, len = 9;
    const uint8_t *pl;
    CHECK(rc_frame_decode(buf, n, &type, &pl, &len));
    CHECK(type == RC_FRAME_HELLO && len == 0);

    RcFrameParser p;
    rc_frame_parser_init(&p);
    CHECK(feed(&p, buf, n) == 1 && p.type == RC_FRAME_HELLO);
}

static void test_encode_limits(void)
{
    uint8_t big[RC_FRAME_MAX_PAYLOAD + 1] = { 0 };
    uint8_t buf[RC_FRAME_MAX + 4];
    CHECK(rc_frame_encode(RC_FRAME_ACK, big, sizeof(big), buf, sizeof(buf)) == 0);
    CHECK(rc_frame_encode(RC_FRAME_PACKET, &PKT, sizeof(PKT), buf, 8) == 0);   /* out too small */
}

static void test_decode_rejects(void)
{
    uint8_t buf[RC_FRAME_MAX];
    const size_t n = rc_frame_encode(RC_FRAME_PACKET, &PKT, sizeof(PKT), buf, sizeof(buf));
    uint8_t type, len;
    const uint8_t *pl;

    uint8_t bad[RC_FRAME_MAX];
    memcpy(bad, buf, n);
    bad[6] ^= 0x01;                                   /* payload bit flip */
    CHECK(!rc_frame_decode(bad, n, &type, &pl, &len));

    CHECK(!rc_frame_decode(buf, n - 1, &type, &pl, &len));   /* truncated  */
    CHECK(!rc_frame_decode(buf, 3, &type, &pl, &len));

    memcpy(bad, buf, n);
    bad[0] = 0x00;                                    /* wrong sync */
    CHECK(!rc_frame_decode(bad, n, &type, &pl, &len));

    uint8_t longer[RC_FRAME_MAX + 1];
    memcpy(longer, buf, n);
    longer[n] = 0;                                    /* trailing junk */
    CHECK(!rc_frame_decode(longer, n + 1, &type, &pl, &len));
}

static void test_stream_resync(void)
{
    uint8_t f1[RC_FRAME_MAX], f2[RC_FRAME_MAX];
    const RcAck ack = { .seq = 3, .arm_state = 1, .mode = 1, .link_pct = 98, .throttle_us = 1350 };
    const size_t n1 = rc_frame_encode(RC_FRAME_PACKET, &PKT, sizeof(PKT), f1, sizeof(f1));
    const size_t n2 = rc_frame_encode(RC_FRAME_ACK, &ack, sizeof(ack), f2, sizeof(f2));

    /* Junk, a frame cut off half way (bridge reset), then two good frames. */
    uint8_t stream[128];
    size_t k = 0;
    const uint8_t junk[] = { 0x00, 0xFF, 0xA5, 0x13, 0x5A, 0xA5 };
    memcpy(&stream[k], junk, sizeof(junk)); k += sizeof(junk);
    memcpy(&stream[k], f1, n1 / 2);         k += n1 / 2;
    memcpy(&stream[k], f1, n1);             k += n1;
    memcpy(&stream[k], f2, n2);             k += n2;

    RcFrameParser p;
    rc_frame_parser_init(&p);
    int got = 0, acks = 0, pkts = 0;
    for (size_t i = 0; i < k; i++) {
        if (rc_frame_parse_byte(&p, stream[i])) {
            got++;
            if (p.type == RC_FRAME_ACK && p.len == sizeof(ack) && memcmp(p.payload, &ack, sizeof(ack)) == 0) acks++;
            if (p.type == RC_FRAME_PACKET && p.len == sizeof(PKT) && memcmp(p.payload, &PKT, sizeof(PKT)) == 0) pkts++;
        }
    }
    /* The half frame swallows the start of the next one at worst — the
     * parser must still lock on by the last frame. */
    CHECK(acks == 1);
    CHECK(got >= 1 && got <= 2);
    CHECK(pkts <= 1);

    /* After the damage, clean frames always come through. */
    CHECK(feed(&p, f1, n1) == 1 && p.type == RC_FRAME_PACKET);
    CHECK(feed(&p, f2, n2) == 1 && p.type == RC_FRAME_ACK);
}

static void test_double_sync(void)
{
    /* A5 A5 5A ...: the second A5 starts the frame. */
    uint8_t buf[RC_FRAME_MAX + 1];
    buf[0] = RC_FRAME_SYNC0;
    const size_t n = rc_frame_encode(RC_FRAME_PACKET, &PKT, sizeof(PKT), &buf[1], RC_FRAME_MAX);
    RcFrameParser p;
    rc_frame_parser_init(&p);
    CHECK(feed(&p, buf, n + 1) == 1);
}

static void test_bad_crc_and_length_counted(void)
{
    uint8_t buf[RC_FRAME_MAX];
    const size_t n = rc_frame_encode(RC_FRAME_PACKET, &PKT, sizeof(PKT), buf, sizeof(buf));
    RcFrameParser p;
    rc_frame_parser_init(&p);

    buf[n - 1] ^= 0xFF;
    CHECK(feed(&p, buf, n) == 0);
    CHECK(p.errors == 1);

    const uint8_t too_long[] = { RC_FRAME_SYNC0, RC_FRAME_SYNC1, RC_FRAME_PACKET, RC_FRAME_MAX_PAYLOAD + 1 };
    CHECK(feed(&p, too_long, sizeof(too_long)) == 0);
    CHECK(p.errors == 2);
}

static void test_every_single_bit_flip_rejected(void)
{
    uint8_t buf[RC_FRAME_MAX];
    const size_t n = rc_frame_encode(RC_FRAME_PACKET, &PKT, sizeof(PKT), buf, sizeof(buf));
    int accepted = 0;
    for (size_t byte = 2; byte < n; byte++) {        /* past the sync bytes */
        for (int bit = 0; bit < 8; bit++) {
            uint8_t t[RC_FRAME_MAX];
            memcpy(t, buf, n);
            t[byte] ^= (uint8_t)(1u << bit);
            uint8_t type, len;
            const uint8_t *pl;
            if (rc_frame_decode(t, n, &type, &pl, &len)) accepted++;
        }
    }
    CHECK(accepted == 0);
}

static void test_frame_fits_espnow(void)
{
    /* ESP-NOW v1 carries up to 250 bytes; our biggest frame is far below. */
    CHECK(sizeof(RcPacket) <= RC_FRAME_MAX_PAYLOAD);
    CHECK(sizeof(RcAck) <= RC_FRAME_MAX_PAYLOAD);
    CHECK(RC_FRAME_MAX <= 250);
}

int main(void)
{
    test_roundtrip();
    test_empty_payload();
    test_encode_limits();
    test_decode_rejects();
    test_stream_resync();
    test_double_sync();
    test_bad_crc_and_length_counted();
    test_every_single_bit_flip_rejected();
    test_frame_fits_espnow();

    printf("%d checks, %d failed\n", s_checks, s_failed);
    return s_failed ? 1 : 0;
}
