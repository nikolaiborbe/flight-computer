#pragma once

#include "capacitance_sensor.hpp"
#include "fdc1004.hpp"
#include "ring_buffer.hpp"
#include "mutex.hpp"

namespace sensors::capacitance {

class LocalCapacitanceSensor : public CapacitanceSensor {
public:

  LocalCapacitanceSensor(Fdc1004& fdc1004_driver, Fdc1004Measurement measurement, Fdc1004Channel channel,
    const SensorConfig& config);

  void init();

  float getCapacitance() const override;

  FaultFlags getFaults() const override;

  SensorConfig getConfig() const override;

  void update(uint64_t now_us) override;

  bool popSamples(CapacitanceBatch& out) override;

private:
  void configureMeasurement();

  void pushSample(uint64_t t_us);

  void flushBatch();

  static int16_t pfToCpf(float pf);

  mutable Mutex mutex_{};

  Fdc1004& fdc1004_driver_;
  Fdc1004Measurement measurement_;
  Fdc1004Channel channel_;
  SensorConfig config_{};

  float offset_pf_ = 0.0f;
  float raw_measurement_pf_ = 0.0f;
  float adjusted_measurement_pf_ = 0.0f;

  bool discard_next_sample_ = false;

  FaultFlags faults_{0};

  uint64_t last_update_us_ = 0;
  uint64_t next_sample_due_us_ = 0;

  CapacitanceBatch current_batch_{};
  uint8_t batch_seq_ = 0;

  RingBuffer<CapacitanceBatch, 8> batch_buffer_{};
};

}
