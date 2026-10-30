#pragma once

#include <cstdint>

namespace sensors::capacitance {

inline constexpr std::size_t BatchSampleCount = 8;

enum class Fault : uint16_t {
  None          = 0,
  InvalidConfig = 1 << 0,
  OutOfRange    = 1 << 1,
  CommFault     = 1 << 2,
  Stale         = 1 << 3,
  DataLoss      = 1 << 4,
  FdcFault      = 1 << 5,
};

using FaultFlags = uint16_t;

constexpr FaultFlags toFlags(Fault f) {
  return static_cast<FaultFlags>(f);
}

struct SensorConfig {

};

/**
 * #packed
 */
struct CapacitanceSample {
  int16_t offset_cpf = 0;
  int16_t raw_cpf = 0;
  int16_t adjusted_cpf = 0;
};

/**
 * Batched capacitance telemetry.
 *
 * #packed
 */
struct CapacitanceBatch {
  uint64_t t0_us = 0;
  uint32_t dt_us = 0;
  uint8_t count = 0;
  uint8_t seq = 0;
  CapacitanceSample samples[BatchSampleCount]{};
};

class CapacitanceSensor {
public:
  virtual ~CapacitanceSensor() = default;

  virtual float getCapacitance() const = 0;

  virtual FaultFlags getFaults() const = 0;

  virtual SensorConfig getConfig() const = 0;

  virtual void update(uint64_t now_us) = 0;
  
  virtual bool popSamples(CapacitanceBatch& out) = 0;
};

}
