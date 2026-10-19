/*
 * Host tests for the radio link supervisor (rc_link.c).
 *
 *   make -C Core/test
 */
#include "rc_link.h"
#include <math.h>
#include <stdio.h>

static int s_fail, s_checks;

#define CHECK(cond) do { s_checks++; if (!(cond)) { \
    s_fail++; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)

static RcPacket pkt(uint8_t seq, uint16_t thr)
{
    RcPacket p = { .seq = seq, .throttle = thr };
    return p;
}

/* Feed packets every RC_PERIOD_MS from t0 for `ms`, keeping `keep` of every 10. */
static uint32_t feed(RcLink *l, uint32_t t0, uint32_t ms, int keep)
{
    uint32_t t = t0;
    uint8_t seq = 0;
    for (; t < t0 + ms; t += RC_PERIOD_MS, seq++) {
        if (seq % 10 < keep) {
            const RcPacket p = pkt(seq, 0);
            RcLink_OnPacket(l, &p, t);
        }
        RcLink_Update(l, t);
    }
    return t;
}

static void test_none_until_first_packet(void)
{
    RcLink l;
    RcLink_Init(&l, 1000);
    CHECK(RcLink_Update(&l, 1000) == RC_LINK_NONE);
    CHECK(RcLink_Update(&l, 60000) == RC_LINK_NONE);   /* no timeout from NONE */
    CHECK(l.link_pct == 0);
}

static void test_up_then_lost_then_back(void)
{
    RcLink l;
    RcLink_Init(&l, 0);
    const RcPacket p = pkt(7, 300);
    RcLink_OnPacket(&l, &p, 100);
    CHECK(RcLink_Update(&l, 100) == RC_LINK_UP);
    CHECK(l.last.seq == 7 && l.last.throttle == 300);

    /* Exactly at the timeout: still up. One ms later: lost. */
    CHECK(RcLink_Update(&l, 100 + RC_LINK_TIMEOUT_MS) == RC_LINK_UP);
    CHECK(RcLink_Update(&l, 101 + RC_LINK_TIMEOUT_MS) == RC_LINK_LOST);
    CHECK(l.link_pct == 0);
    CHECK(RcLink_Update(&l, 5000) == RC_LINK_LOST);

    const RcPacket q = pkt(8, 0);
    RcLink_OnPacket(&l, &q, 5010);
    CHECK(RcLink_Update(&l, 5010) == RC_LINK_UP);
    CHECK(l.last.seq == 8);
}

static void test_link_pct(void)
{
    RcLink l;
    RcLink_Init(&l, 0);
    uint32_t t = feed(&l, 0, 2000, 10);                 /* every packet */
    CHECK(l.link_pct == 100);

    t = feed(&l, t, 2000, 5);                           /* half of them */
    CHECK(l.link_pct == 50);

    t = feed(&l, t, 2000, 9);                           /* 90 % */
    CHECK(l.link_pct == 90);
    CHECK(RcLink_Update(&l, t) == RC_LINK_UP);
}

static void test_link_pct_after_long_gap(void)
{
    RcLink l;
    RcLink_Init(&l, 0);
    const uint32_t t = feed(&l, 0, 2000, 10);
    CHECK(l.link_pct == 100);
    /* Nothing polled for 3 s, then one packet: the stale window must not
     * report the old count. */
    const RcPacket p = pkt(1, 0);
    RcLink_OnPacket(&l, &p, t + 3000);
    RcLink_Update(&l, t + 3000);
    CHECK(l.link_pct == 0);
}

static void test_link_pct_capped(void)
{
    RcLink l;
    RcLink_Init(&l, 0);
    for (uint32_t t = 0; t < 1000; t += 5) {            /* 4x the expected rate */
        const RcPacket p = pkt(0, 0);
        RcLink_OnPacket(&l, &p, t);
    }
    RcLink_Update(&l, 1000);
    CHECK(l.link_pct == 100);
}

static void test_normalize(void)
{
    CHECK(RcLink_Axis(0) == 0.0f);
    CHECK(RcLink_Axis(RC_AXIS_MAX) == 1.0f);
    CHECK(RcLink_Axis(-RC_AXIS_MAX) == -1.0f);
    CHECK(fabsf(RcLink_Axis(500) - 0.5f) < 1e-6f);
    CHECK(RcLink_Axis(32767) == 1.0f);                  /* garbage clamps */
    CHECK(RcLink_Axis(-32768) == -1.0f);

    CHECK(RcLink_Throttle(0) == 0.0f);
    CHECK(RcLink_Throttle(RC_THROTTLE_MAX) == 1.0f);
    CHECK(fabsf(RcLink_Throttle(250) - 0.25f) < 1e-6f);
    CHECK(RcLink_Throttle(65535) == 1.0f);
}

static void test_tick_wraparound(void)
{
    RcLink l;
    const uint32_t t0 = 0xFFFFFF00u;
    RcLink_Init(&l, t0);
    const RcPacket p = pkt(1, 0);
    RcLink_OnPacket(&l, &p, t0 + 0x80u);
    CHECK(RcLink_Update(&l, t0 + 0x80u) == RC_LINK_UP);
    CHECK(RcLink_Update(&l, t0 + 0x80u + 200u) == RC_LINK_UP);  /* wraps past 0 */
    CHECK(RcLink_Update(&l, t0 + 0x80u + 300u) == RC_LINK_LOST);
}

int main(void)
{
    test_none_until_first_packet();
    test_up_then_lost_then_back();
    test_link_pct();
    test_link_pct_after_long_gap();
    test_link_pct_capped();
    test_normalize();
    test_tick_wraparound();

    printf("%s: %d checks, %d failed\n", s_fail ? "FAIL" : "OK", s_checks, s_fail);
    return s_fail ? 1 : 0;
}
