#include "servo_model.hpp"

namespace actuators::servo {

inline constexpr ServoModel SERVO_MODEL_UNKNOWN {
  ServoModelId::Unknown,
  AngleLimits{0.0f, 0.0f},
  AngleCalibration{0, 0},
  PulseRange{1500u, 1500u},
  0u,
  0.0f
};

inline constexpr ServoModel SERVO_MODEL_T70 {
  ServoModelId::PowerHD_T70_12V,
  AngleLimits{0.0f, 90.0f},
  AngleCalibration{2.35932f, 0.99118f},
  PulseRange{1000u, 2000u},
  500u,
  0.1f
};

const ServoModel& getServoModel(const ServoModelId id) {
  switch (id) {
    case ServoModelId::PowerHD_T70_12V:
      return SERVO_MODEL_T70;
    default:
      return SERVO_MODEL_UNKNOWN;
  }
}

}
