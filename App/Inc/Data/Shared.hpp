#ifndef TELEMETRY_SHARED_DATA_HPP
#define TELEMETRY_SHARED_DATA_HPP

#include "borbes_shorthands.h"

#pragma pack(push, 1)

namespace Telemetry::Shared {
typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;
} AccelData;

typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
} GyroData;

typedef struct {
    float acc_x_m_per_ss;
    float acc_y_m_per_ss;
    float acc_z_m_per_ss;
} AccelerationData;

typedef struct {
    float mag_x_gauss;
    float mag_y_gauss;
    float mag_z_gauss;
} MagnetometerData;

typedef struct {
    float temperature_C;
    GyroData gyro;
    AccelerationData acc;
    ULONG timestamp;
} ImuData;
} // namespace Telemetry::Shared

#pragma pack(pop)
#endif
