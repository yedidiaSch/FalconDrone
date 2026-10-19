/*
 * Host tests for the step-test sequencer (tune_seq.c).
 *
 *   make -C Core/test
 */
#include "tune_seq.h"
#include <stdio.h>

static int s_fail, s_checks;

#define CHECK(cond) do { s_checks++; if (!(cond)) { \
    s_fail++; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)

static uint8_t at(TuneSeq *s, uint32_t t, float *ref, uint8_t *seg)
{
    return TuneSeq_Step(s, t, ref, seg);
}

static void test_doublet_shape(void)
{
    const TuneSeqCfg cfg = { .amp = 60.0f, .step_ms = 300, .reps = 2 };
    TuneSeq s;
    TuneSeq_Start(&s, &cfg, 1000);
    float ref; uint8_t seg;

    /* hold = max(300, 500) = 500; rep = 2 * (300 + 500) = 1600 */
    CHECK(TuneSeq_DurationMs(&cfg) == 500u + 2u * 1600u);

    CHECK(at(&s, 1000, &ref, &seg) == 1 && ref == 0.0f && seg == 0);   /* lead-in */
    CHECK(at(&s, 1499, &ref, &seg) == 1 && ref == 0.0f && seg == 0);
    CHECK(at(&s, 1500, &ref, &seg) == 1 && ref == 60.0f && seg == 1);  /* +step */
    CHECK(at(&s, 1799, &ref, &seg) == 1 && ref == 60.0f && seg == 1);
    CHECK(at(&s, 1800, &ref, &seg) == 1 && ref == 0.0f && seg == 2);   /* hold */
    CHECK(at(&s, 2299, &ref, &seg) == 1 && ref == 0.0f && seg == 2);
    CHECK(at(&s, 2300, &ref, &seg) == 1 && ref == -60.0f && seg == 3); /* -step */
    CHECK(at(&s, 2600, &ref, &seg) == 1 && ref == 0.0f && seg == 4);   /* hold */
    CHECK(at(&s, 3100, &ref, &seg) == 1 && ref == 60.0f && seg == 5);  /* rep 2 */
    CHECK(at(&s, 3900, &ref, &seg) == 1 && ref == -60.0f && seg == 7);
    CHECK(at(&s, 4699, &ref, &seg) == 1 && ref == 0.0f && seg == 8);   /* last ms */
    CHECK(at(&s, 4700, &ref, &seg) == 0 && ref == 0.0f && seg == 0);   /* done */
    CHECK(s.active == 0);
    CHECK(at(&s, 4800, &ref, &seg) == 0);
}

static void test_long_step_sets_hold(void)
{
    const TuneSeqCfg cfg = { .amp = -10.0f, .step_ms = 800, .reps = 1 };
    TuneSeq s;
    TuneSeq_Start(&s, &cfg, 0);
    float ref; uint8_t seg;
    CHECK(TuneSeq_DurationMs(&cfg) == 500u + 2u * (800u + 800u));
    CHECK(at(&s, 500, &ref, &seg) == 1 && ref == -10.0f && seg == 1);  /* sign kept */
    CHECK(at(&s, 1300, &ref, &seg) == 1 && ref == 0.0f && seg == 2);
    CHECK(at(&s, 2100, &ref, &seg) == 1 && ref == 10.0f && seg == 3);
    CHECK(at(&s, 2900, &ref, &seg) == 1 && ref == 0.0f && seg == 4);
    CHECK(at(&s, 3700, &ref, &seg) == 0);
}

static void test_stop_and_wraparound(void)
{
    const TuneSeqCfg cfg = { .amp = 30.0f, .step_ms = 100, .reps = 10 };
    TuneSeq s;
    float ref; uint8_t seg;

    TuneSeq_Start(&s, &cfg, 0xFFFFFE00u);                 /* tick wraps mid-run */
    CHECK(at(&s, 0xFFFFFE00u + 500u, &ref, &seg) == 1 && ref == 30.0f && seg == 1);
    CHECK(at(&s, 0xFFFFFE00u + 500u + 1200u * 9u, &ref, &seg) == 1 && seg == 37);

    TuneSeq_Stop(&s);
    CHECK(at(&s, 0xFFFFFE00u + 600u, &ref, &seg) == 0 && ref == 0.0f && seg == 0);
}

int main(void)
{
    test_doublet_shape();
    test_long_step_sets_hold();
    test_stop_and_wraparound();
    printf("%s: %d checks, %d failed\n", s_fail ? "FAIL" : "OK", s_checks, s_fail);
    return s_fail ? 1 : 0;
}
