#pragma once

#include "mutex.hpp"
#include "pressure_sensor.hpp"
#include "ring_buffer.hpp"

namespace sensors::pressure {

struct AdcSample {
  uint64_t t_us = 0;
  float volts = 0.0f;
};

class AdcPressureSensor final : public PressureSensor {
public:
  explicit AdcPressureSensor(const SensorConfig& config);

  float getPressure() const override;

  FaultFlags getFaults() const override;

  void setSamplePeriod(uint32_t sample_period_us) override;

  void setSensorConfig(const SensorConfig& config);

  SensorConfig getConfig() const override;

  void update(uint64_t now_us) override;

  void onSample(uint64_t t_us, float volts);

  bool popSamples(PressureBatch& out) override;

private:
  void processSample(uint64_t t_us, float volts);

  void pushSample(uint64_t t_us, int16_t pressure_cbar);

  void flushBatch();

  static float mapLinear(float voltage, float x0, float v_max, float bar_min, float bar_max);

  FaultFlags validateConfig(const SensorConfig& cfg);

  mutable Mutex mutex_{};

  float pressure_bar_{0.0f};
  FaultFlags faults_{0};

  uint64_t last_update_us_{0};
  uint64_t last_sample_us_{0};
  uint64_t next_sample_due_us_{0};

  SensorConfig config_{};

  RingBuffer<AdcSample, 32> adc_buffer_{};
  RingBuffer<PressureBatch, 8> batch_buffer_{};

  PressureBatch current_batch_{};
  uint8_t batch_seq_{0};
};

}
