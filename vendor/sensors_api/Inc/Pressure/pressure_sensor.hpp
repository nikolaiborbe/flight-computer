/**
 * @file pressure_sensor.hpp
 * @author Henrik Berg (henrik.berg@propulsentnu.no)
 *
 * @brief Pressure sensor interface and shared data types.
 *
 * Defines the common pressure sensor API used by both locally
 * connected and remotely connected pressure sensors.
 *
 * @copyright Propulse NTNU (c) 2026
 */
#pragma once

namespace sensors::pressure {

inline constexpr std::size_t BatchSampleCount = 20;

/**
 * Electrical output type of the pressure sensor.
 */
enum class OutputType : uint8_t {
  Voltage = 0,
  Current = 1
};

/**
 * Pressure sensor fault flags. Multiple flags may be active simultaneously.
 */
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

/**
 * Pressure sensor configuration.
 *
 * Defines how an electrical sensor output maps to a pressure range.
 *
 * #packed
 */
struct SensorConfig {
  float p_min_bar = 0.0f;
  float p_max_bar = 0.0f;

  OutputType output_type = OutputType::Voltage;

  float v_min = 0.0f;
  float v_max = 0.0f;

  float i_min_ma = 4.0f;
  float i_max_ma = 20.0f;
  float shunt_ohms = 165.0f;

  float disconnect_threshold = 0.0f;
  float margin = 0.05f;

  uint32_t sample_period_us = 5000;
};

/**
 * Batched pressure telemetry.
 *
 * #packed
 */
struct PressureBatch {
  uint64_t t0_us = 0;
  uint32_t dt_us = 0;
  uint8_t count = 0;
  uint8_t seq = 0;
  int16_t samples[BatchSampleCount]{};
};

/**
 * Generic pressure sensor interface.
 */
class PressureSensor {
public:
  virtual ~PressureSensor() = default;

  /**
   * Returns the most recent pressure measurement.
   *
   * @return Pressure in bar.
   */
  virtual float getPressure() const = 0;

  /**
   * Returns active sensor fault flags.
   *
   * @return Bitmask of active Fault values.
   */
  virtual FaultFlags getFaults() const = 0;

  /**
   * Updates the sample period.
   *
   * @param sample_period_us Sample period in microseconds.
   */
  virtual void setSamplePeriod(uint32_t sample_period_us) = 0;

  /**
   * Returns the current sensor configuration.
   *
   * @return Sensor configuration.
   */
  virtual SensorConfig getConfig() const = 0;

  virtual void update(uint64_t now_us) = 0;

  virtual bool popSamples(PressureBatch& out) = 0;
};

}
