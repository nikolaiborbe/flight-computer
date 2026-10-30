#ifndef NAMESPACE_SENSOR_ICM_HPP
#define NAMESPACE_SENSOR_ICM_HPP

// App
#include "ICM40609D.h"
#include "IMU.h"
#include "main.h"

// stm
#include "spi.h"

// TODO: Should be in sesnors_api. This is a temp fix
namespace Driver::Sensors {
static constexpr float ACC_OFFSET_X_G = 0.0f / static_cast<float>(G);
static constexpr float ACC_OFFSET_Y_G = 0.0f / static_cast<float>(G);
static constexpr float ACC_OFFSET_Z_G = 0.0f / static_cast<float>(G);
namespace ICM {
constexpr Orientation orientation = Orientation {0.f, static_cast<float>(PI / 2.0), 0.f};
constexpr Orientation orientation2 = Orientation {static_cast<float>(PI / 4.0), 0.f, 0.f};

ICM40609D init();
} // namespace ICM
} // namespace Driver::Sensors

#endif // !CLASS_ICM_HPP
