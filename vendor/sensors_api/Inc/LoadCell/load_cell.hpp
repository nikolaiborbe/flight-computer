#pragma once

#include <cstdint>

namespace sensors::load_cell {

inline constexpr std::size_t BatchSampleCount = 20;

enum class Fault : uint16_t {
  None          = 0,
  Disconnected  = 1 << 0,
  InvalidConfig = 1 << 1,
  OutOfRange    = 1 << 2,
  CommFault     = 1 << 3,
  Stale         = 1 << 4,
  DataLoss      = 1 << 5
};

using FaultFlags = uint16_t;

constexpr FaultFlags toFlags(Fault fault) {
  return static_cast<FaultFlags>(fault);
}

struct SensorConfig {
  float capacity_kg = 1000.0f;

  float scale = 1.0f;      // kg / V
  float offset = 0.0f;     // kg
  float zero_volts = 1.5f; // INA output at zero load

  float min_valid = -1e9f;
  float max_valid =  1e9f;

  uint32_t sample_period_us = 5000;
};

#pragma pack(push, 1)
struct LoadCellBatch {
  uint64_t t0_us = 0;
  uint32_t dt_us = 0;
  uint8_t count = 0;
  uint8_t seq = 0;
  int16_t samples[BatchSampleCount]{};
};
#pragma pack(pop)

class LoadCell {
public:
  virtual ~LoadCell() = default;

  virtual float getLoad() const = 0;

  virtual FaultFlags getFaults() const = 0;

  virtual void setSamplePeriod(uint32_t sample_period_us) = 0;

  virtual SensorConfig getConfig() const = 0;

  virtual void update(uint64_t now_us) = 0;

  virtual bool popSamples(LoadCellBatch& out) = 0;
};

}