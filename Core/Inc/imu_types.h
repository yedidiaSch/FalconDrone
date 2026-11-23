/**
 * @file imu_types.h
 * @brief Sensor-independent IMU sample types, shared by the IMU drivers
 *        (MPU6050, ICM20948) and the attitude filter.
 *
 * @author Yedidya Schwartz
 */

#ifndef IMU_TYPES_H_
#define IMU_TYPES_H_

/** One decoded sensor read. Accel in g, gyro in dps with bias removed. */
struct ImuSample {
    float ax, ay, az;
    float gx, gy, gz;
};

/** Per-axis gyro zero-rate offset in dps. */
struct GyroBias {
    float x, y, z;
};

#endif // IMU_TYPES_H_
