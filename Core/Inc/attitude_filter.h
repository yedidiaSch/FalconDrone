/**
 * @file attitude_filter.h
 * @brief Roll/pitch estimation from IMU samples (complementary filter).
 *
 * Kept separate from the MPU6050 driver so the estimator can be swapped
 * (Madgwick / Kalman) without touching sensor I/O.
 *
 * @author Yedidya Schwartz
 */

#ifndef ATTITUDE_FILTER_H_
#define ATTITUDE_FILTER_H_

#include "MPU6050.h"

class ComplementaryFilter {
public:
    /** @param alpha Gyro weight per step (0.98 → ~0.5 s accel time constant at 100 Hz). */
    explicit constexpr ComplementaryFilter(float alpha) : _alpha(alpha) {}

    /**
     * @brief Fuse one sample.
     * @param s  Decoded sensor sample.
     * @param dt Seconds since the previous sample; 0 skips gyro integration.
     */
    void Update(const ImuSample& s, float dt);

    float Roll() const  { return _roll; }   /**< degrees */
    float Pitch() const { return _pitch; }  /**< degrees */

private:
    float _alpha;
    float _roll = 0.0f;
    float _pitch = 0.0f;
    bool _seeded = false;
};

#endif // ATTITUDE_FILTER_H_
