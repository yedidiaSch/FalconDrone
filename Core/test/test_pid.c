/*
 * Host tests for the PID controller (pid.c), mainly the derivative
 * low-pass added for the 500 Hz loop.
 *
 *   make -C Core/test
 */
#include "pid.h"
#include <math.h>
#include <stdio.h>

static int s_fail, s_checks;

#define CHECK(cond) do { s_checks++; if (!(cond)) { \
    s_fail++; printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)

#define NEAR(a, b, tol) (fabsf((a) - (b)) <= (tol))

/* D only: output = -kd * d(measurement)/dt. */
static PID_t d_only(float lpf_hz)
{
    PID_t p;
    PID_Init(&p, 0.0f, 0.0f, 1.0f, 0.0f, 1e9f);
    PID_SetDLpf(&p, lpf_hz);
    return p;
}

static void test_unfiltered_is_raw_difference(void)
{
    PID_t p = d_only(0.0f);
    CHECK(PID_Update(&p, 0.0f, 5.0f, 0.002f) == 0.0f);        /* first call: no D */
    CHECK(NEAR(PID_Update(&p, 0.0f, 6.0f, 0.002f), -500.0f, 1e-2f));  /* 1 / 2 ms */
    CHECK(NEAR(PID_Update(&p, 0.0f, 6.0f, 0.002f), 0.0f, 1e-6f));
}

/* A ramp has a constant derivative: the filter must settle on it exactly. */
static void test_filter_passes_dc(void)
{
    PID_t p = d_only(40.0f);
    float out = 0.0f;
    for (int i = 0; i < 500; i++) {                /* 1 s at 500 Hz, 100 dps/s */
        out = PID_Update(&p, 0.0f, 0.2f * (float)i, 0.002f);
    }
    CHECK(NEAR(out, -100.0f, 0.01f));
}

/* Alternating +/-1 dps noise at 250 Hz (Nyquist): raw D is +/-1000; a
 * 40 Hz first-order filter must cut it by well over 4x. */
static void test_filter_cuts_nyquist_noise(void)
{
    PID_t raw = d_only(0.0f), filt = d_only(40.0f);
    float max_raw = 0.0f, max_filt = 0.0f;
    for (int i = 0; i < 500; i++) {
        const float m = (i & 1) ? 1.0f : -1.0f;
        const float r = fabsf(PID_Update(&raw, 0.0f, m, 0.002f));
        const float f = fabsf(PID_Update(&filt, 0.0f, m, 0.002f));
        if (i > 100) {                             /* after settling */
            if (r > max_raw)  max_raw = r;
            if (f > max_filt) max_filt = f;
        }
    }
    CHECK(NEAR(max_raw, 1000.0f, 1e-2f));
    CHECK(max_filt < max_raw / 4.0f);
}

/* The cutoff is in Hz: a derivative step must rise on the same time scale
 * at 100 Hz and at 500 Hz. 63% at one time constant (1 / (2 pi 40) = 4 ms). */
static float response_at(float dt, float t_end)
{
    PID_t p = d_only(40.0f);
    PID_Update(&p, 0.0f, 0.0f, dt);                /* prime prev_measurement */
    float out = 0.0f, m = 0.0f;
    for (float t = 0.0f; t < t_end - 1e-6f; t += dt) {
        m += 100.0f * dt;                          /* ramp starts: d = 100 */
        out = PID_Update(&p, 0.0f, m, dt);
    }
    return -out / 100.0f;
}

static void test_cutoff_independent_of_rate(void)
{
    const float at500 = response_at(0.002f, 0.020f);
    const float at100 = response_at(0.010f, 0.020f);
    CHECK(at500 > 0.9f && at500 < 1.0f);           /* 5 tau → ~99% */
    CHECK(fabsf(at500 - at100) < 0.1f);
}

static void test_reset_clears_filter(void)
{
    PID_t p = d_only(40.0f);
    PID_Update(&p, 0.0f, 0.0f, 0.002f);
    PID_Update(&p, 0.0f, 10.0f, 0.002f);
    CHECK(p.d_filt != 0.0f);
    PID_Reset(&p);
    CHECK(p.d_filt == 0.0f && p.has_prev == 0u);
    CHECK(PID_Update(&p, 0.0f, 50.0f, 0.002f) == 0.0f);   /* no kick after reset */
}

static void test_p_and_i(void)
{
    PID_t p;
    PID_Init(&p, 2.0f, 10.0f, 0.0f, 0.05f, 100.0f);
    CHECK(NEAR(PID_Update(&p, 10.0f, 0.0f, 0.002f), 20.0f + 10.0f * 0.02f, 1e-4f));
    for (int i = 0; i < 1000; i++) PID_Update(&p, 10.0f, 0.0f, 0.002f);
    CHECK(NEAR(p.integral, 0.05f, 1e-6f));         /* anti-windup clamp */
    CHECK(NEAR(PID_Update(&p, 1000.0f, 0.0f, 0.002f), 100.0f, 1e-4f));  /* out clamp */
}

int main(void)
{
    test_unfiltered_is_raw_difference();
    test_filter_passes_dc();
    test_filter_cuts_nyquist_noise();
    test_cutoff_independent_of_rate();
    test_reset_clears_filter();
    test_p_and_i();
    printf("%s: %d checks, %d failed\n", s_fail ? "FAIL" : "OK", s_checks, s_fail);
    return s_fail != 0;
}
