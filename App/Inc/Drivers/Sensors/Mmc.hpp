#ifndef NAMESPACE_SENSOR_MMC_HPP
#define NAMESPACE_SENSOR_MMC_HPP

// App
#include "IMU.h"
#include "MMC5983MA.h"
#include "main.h"

// TODO: Should be in sesnors_api. This is a temp fix
namespace Driver::Sensors {
namespace MMC {
constexpr Orientation orientation = Orientation {0.f, static_cast<float>(PI / 2.0), 0.f};
MMC5983MA init();
void linear_transform_mag(Mmc5983maData& data);
} // namespace MMC
} // namespace Driver::Sensors

#endif // !NAMESPACE_SENSOR_MMC_HPP
