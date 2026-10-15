/**
 * @file attitude_filter.cpp
 * @brief Complementary filter implementation.
 *
 * @author Yedidya Schwartz
 */

#include "attitude_filter.h"
#include <cmath>

namespace {
constexpr float kRadToDeg = 57.29577951f;
}

void ComplementaryFilter::Update(const ImuSample& s, float dt)
{
    // Static angle from gravity. std:: overloads with float args resolve to
    // atan2f/sqrtf — the M4F FPU is single precision, double would be
    // software-emulated.
    const float accel_roll  = std::atan2(s.ay, s.az) * kRadToDeg;
    const float accel_pitch = std::atan2(-s.ax, std::sqrt(s.ay * s.ay + s.az * s.az)) * kRadToDeg;

    // First sample: start from the accel angle instead of converging from 0.
    if (!_seeded) {
        _roll = accel_roll;
        _pitch = accel_pitch;
        _seeded = true;
        return;
    }

    _roll  = _alpha * (_roll  + s.gx * dt) + (1.0f - _alpha) * accel_roll;
    _pitch = _alpha * (_pitch + s.gy * dt) + (1.0f - _alpha) * accel_pitch;
}
