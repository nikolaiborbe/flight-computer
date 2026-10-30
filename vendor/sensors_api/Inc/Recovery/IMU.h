/**
 * @file IMU.hpp
 *
 * @author Nikolai G. Borbe (nikolai.borbe@propulsentnu.com)
 * @brief This file contains the base class for the innertial measurement drivers.
 * @version 1.0
 *
 * @date 2025-10-29
 */

#ifndef IMU_H
#define IMU_H

#include "logger.h"
#include "main.h"
#include <stdbool.h>

// deg/s
struct GyroData {
    float gyro_x;
    float gyro_y;
    float gyro_z;
};

// m/s^2
struct AccelerationData {
    float acc_x;
    float acc_y;
    float acc_z;
};

// C
struct TemperatureData {
    float temperature;
};

struct AccelerationOffset {
    float acc_offset_x;
    float acc_offset_y;
    float acc_offset_z;
};

#ifdef __cplusplus
#include "quaternion.h"
#include <stdbool.h>

#include <array>
#include <cmath>
#include <cstdint>

constexpr uint32_t SPI_TIMEOUT_MS = 100;
constexpr double G =
    9.8214599692; // https://www.researchgate.net/publication/232822567_Absolute_gravity_values_in_Norway
constexpr double PI = 3.141592653589;


struct ImuData {
    bool valid;
    TemperatureData temperature;
    GyroData gyro;
    AccelerationData acc;
    ULONG timestamp;
};

// in radians
struct Orientation {
    float z; // yaw
    float y; // pitch
    float x; // roll
};

enum class DEVICE_STATUS { ON, OFF };

template <typename T> class IMU {
public:
    IMU() { }

protected:
    ImuData m_sensor_data {};
};

namespace IMU_methods {

/**
 * @brief Rotate a measured sensor vector into the rocket frame.
 *
 * @param x X component of the measured vector.
 * @param y Y component of the measured vector.
 * @param z Z component of the measured vector.
 * @param orientation The sensor orientation in euler angles using YAW-PITCH-ROLL (Z-Y'-X'').
 */
void vector_mount_offset(float& x, float& y, float& z, const Orientation& e);

/**
 * @brief Change the measured values depending on the IMU's orientation.
 *
 * @param data Acceleration data.
 * @param orientation The orientation of IIM42652 in euler angles using YAW-PITCH-ROLL (Z-Y'-X'').
 *
 * @note See diagram on page 56:
 * https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000440-iim-42652-typ-v1.1.pdf
 */
void accel_mount_offset(AccelerationData& data, const Orientation& e);

/**
 * @brief Change the measured values depending on the IMU's orientation.
 *
 * @param data Acceleration data.
 * @param orientation The orientation of IIM42652 in euler angles using YAW-PITCH-ROLL (Z-Y'-X'').
 *
 * @note See diagram on page 56:
 * https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000440-iim-42652-typ-v1.1.pdf
 */
void gyro_mount_offset(GyroData& data, const Orientation& e);

} // IMU_methods


#endif
#endif
