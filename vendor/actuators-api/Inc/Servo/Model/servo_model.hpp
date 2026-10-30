#pragma once

#include <cstdint>

namespace actuators::servo {
  
enum class ServoModelId : uint8_t {
  Unknown = 0,
  PowerHD_T70_12V = 1
};

struct AngleLimits {
  float min_deg;
  float max_deg;
};

struct AngleCalibration {
  float v_min_deg;
  float v_max_deg;
};

struct PulseRange {
  uint16_t min_us;
  uint16_t max_us;
};

struct ServoModel {
  ServoModelId id;
  AngleLimits angle_limits;
  AngleCalibration angle_calibration;
  PulseRange pulse_range;
  uint64_t boot_time_ms;
  float sec_per_60_deg;
};

const ServoModel& getServoModel(ServoModelId id);

}
