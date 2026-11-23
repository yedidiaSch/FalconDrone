/**
 * @file attitude_filter.h
 * @brief Roll/pitch estimation from IMU samples (complementary filter).
 *
 * Kept separate from the IMU drivers so the estimator can be swapped
 * (Madgwick / Kalman) without touching sensor I/O.
 *
 * @author Yedidya Schwartz
 */

#ifndef ATTITUDE_FILTER_H_
#define ATTITUDE_FILTER_H_

#include "imu_types.h"

class ComplementaryFilter {
public:
    /**
     * @param tau_s Time constant in seconds: how long the gyro is trusted
     *              before the accelerometer pulls the angle back. The
     *              per-step gyro weight is tau / (tau + dt), so the
     *              behaviour does not change with the loop rate.
     */
    explicit constexpr ComplementaryFilter(float tau_s) : _tau(tau_s) {}

    /**
     * @brief Fuse one sample.
     * @param s  Decoded sensor sample.
     * @param dt Seconds since the previous sample; 0 skips gyro integration.
     */
    void Update(const ImuSample& s, float dt);

    float Roll() const  { return _roll; }   /**< degrees */
    float Pitch() const { return _pitch; }  /**< degrees */

private:
    float _tau;
    float _roll = 0.0f;
    float _pitch = 0.0f;
    bool _seeded = false;
};

#endif // ATTITUDE_FILTER_H_
