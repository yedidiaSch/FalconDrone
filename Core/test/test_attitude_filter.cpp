/*
 * Host tests for the complementary filter (attitude_filter.cpp): the time
 * constant must hold at any loop rate.
 *
 *   make -C Core/test
 */
#include "attitude_filter.h"
#include <cmath>
#include <cstdio>

static int s_fail, s_checks;

#define CHECK(cond) do { s_checks++; if (!(cond)) { \
    s_fail++; std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)

static ImuSample tilted(float roll_deg)
{
    const float r = roll_deg * 3.14159265f / 180.0f;
    return ImuSample{0.0f, std::sin(r), std::cos(r), 0.0f, 0.0f, 0.0f};
}

/* Seed level, then the accelerometer says 10 deg with no gyro rate: the
 * angle must reach 63% of it after one time constant. */
static float after(float dt, float t_end)
{
    ComplementaryFilter f(0.49f);
    f.Update(tilted(0.0f), dt);                   /* seeds at 0 */
    for (float t = 0.0f; t < t_end - 1e-6f; t += dt) {
        f.Update(tilted(10.0f), dt);
    }
    return f.Roll();
}

static void test_time_constant_independent_of_rate()
{
    const float at100 = after(0.010f, 0.49f);
    const float at500 = after(0.002f, 0.49f);
    CHECK(at100 > 6.0f && at100 < 6.6f);          /* ~63% of 10 deg */
    CHECK(std::fabs(at100 - at500) < 0.1f);
}

static void test_seed_and_gap()
{
    ComplementaryFilter f(0.49f);
    f.Update(tilted(20.0f), 0.002f);
    CHECK(std::fabs(f.Roll() - 20.0f) < 1e-3f);   /* first sample seeds */
    f.Update(tilted(-30.0f), 0.0f);
    CHECK(std::fabs(f.Roll() - 20.0f) < 1e-3f);   /* dt = 0 holds */
}

static void test_gyro_integration()
{
    ComplementaryFilter f(1e6f);                  /* ~pure gyro */
    f.Update(tilted(0.0f), 0.002f);
    ImuSample s = tilted(0.0f);
    s.gx = 100.0f;
    for (int i = 0; i < 500; i++) f.Update(s, 0.002f);   /* 1 s at 100 dps */
    CHECK(std::fabs(f.Roll() - 100.0f) < 0.5f);
}

int main()
{
    test_time_constant_independent_of_rate();
    test_seed_and_gap();
    test_gyro_integration();
    std::printf("%s: %d checks, %d failed\n", s_fail ? "FAIL" : "OK", s_checks, s_fail);
    return s_fail != 0;
}
