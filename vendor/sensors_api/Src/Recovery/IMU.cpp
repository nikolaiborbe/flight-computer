#ifndef USE_IMU
#define USE_IMU
#endif
#include "IMU.h"

namespace IMU_methods {

void vector_mount_offset(float& x, float& y, float& z, const Orientation& e)
{
    auto q = Quaternion::fromEuler(e.z, e.y, e.x);

    // we need conjugate because we are transforming from rocket rotation -> sensor rotation.
    q.conjugate().rotateVector(x, y, z);
}

void accel_mount_offset(AccelerationData& data, const Orientation& e)
{
    vector_mount_offset(data.acc_x, data.acc_y, data.acc_z, e);
}

void gyro_mount_offset(GyroData& data, const Orientation& e)
{
    vector_mount_offset(data.gyro_x, data.gyro_y, data.gyro_z, e);
}

} // namespace IMU_methods
