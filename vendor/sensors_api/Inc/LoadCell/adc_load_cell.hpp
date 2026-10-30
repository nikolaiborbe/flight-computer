#pragma once

#include "load_cell.hpp"
#include "mutex.hpp"
#include "ring_buffer.hpp"

namespace sensors::load_cell {

struct AdcSample {
  uint64_t t_us = 0;
  float volts = 0.0f;
};

class AdcLoadCell final : public LoadCell {
public:
  explicit AdcLoadCell(const SensorConfig& config);

  float getLoad() const override;

  FaultFlags getFaults() const override;

  void setSamplePeriod(uint32_t sample_period_us) override;

  SensorConfig getConfig() const override;

  void update(uint64_t now_us) override;

  void onSample(uint64_t now_us, float volts);

  bool popSamples(LoadCellBatch& out) override;

private:
  void processSample(uint64_t t_us, float volts);

  void pushSample(uint64_t t_us, int16_t load_dkg);

  void flush();

  static FaultFlags validateConfig(const SensorConfig& cfg);

  mutable Mutex mutex_{};

  float weight_kg_{0.0f};
  FaultFlags faults_ = toFlags(Fault::None);

  uint64_t last_update_us_{0};
  uint64_t last_sample_us_{0};
  uint64_t next_sample_due_us_{0};

  SensorConfig config_{};

  RingBuffer<AdcSample, 32> adc_buffer_{};
  RingBuffer<LoadCellBatch, 8> batch_buffer_{};

  LoadCellBatch current_batch_{};
  uint8_t batch_seq_{0};
};

}