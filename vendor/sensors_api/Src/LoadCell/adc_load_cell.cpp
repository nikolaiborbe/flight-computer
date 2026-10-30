#include "adc_load_cell.hpp"

#include <cmath>
#include "units.hpp"

namespace sensors::load_cell {

static constexpr uint32_t StaleTimeoutUs = 100000;

AdcLoadCell::AdcLoadCell(const SensorConfig& config)
    : config_(config) {}

float AdcLoadCell::getLoad() const {
  LockGuard lock(mutex_);
  return weight_kg_;
}

FaultFlags AdcLoadCell::getFaults() const {
  LockGuard lock(mutex_);
  return faults_;
}

void AdcLoadCell::setSamplePeriod(uint32_t sample_period_us) {
  LockGuard lock(mutex_);
  config_.sample_period_us = sample_period_us;
  next_sample_due_us_ = 0;
  current_batch_ = {};
}

SensorConfig AdcLoadCell::getConfig() const {
  LockGuard lock(mutex_);
  return config_;
}

void AdcLoadCell::onSample(uint64_t now_us, float volts) {
  if (!adc_buffer_.push({now_us, volts})) {
    LockGuard lock(mutex_);
    faults_ |= toFlags(Fault::DataLoss);
  }
}

void AdcLoadCell::update(uint64_t now_us) {
  AdcSample adc_sample{};

  while (adc_buffer_.pop(adc_sample)) {
    LockGuard lock(mutex_);
    processSample(adc_sample.t_us, adc_sample.volts);
  }

  LockGuard lock(mutex_);

  faults_ &= ~toFlags(Fault::Stale);
  faults_ &= ~toFlags(Fault::InvalidConfig);

  faults_ |= validateConfig(config_);

  if (last_update_us_ != 0 && (now_us - last_update_us_) > StaleTimeoutUs) {
    faults_ |= toFlags(Fault::Stale);
  }
}

void AdcLoadCell::processSample(uint64_t t_us, float volts) {
  if (last_sample_us_ != 0 && t_us < last_sample_us_) {
    next_sample_due_us_ = 0;
    current_batch_ = {};
  }

  last_sample_us_ = t_us;

  if (next_sample_due_us_ == 0) {
    next_sample_due_us_ = t_us;
  }

  if (t_us < next_sample_due_us_) {
    return;
  }

  const uint32_t sample_period = config_.sample_period_us;
  if (sample_period == 0) {
    return;
  }

  next_sample_due_us_ += sample_period;

  if (t_us > next_sample_due_us_ + sample_period) {
    next_sample_due_us_ = t_us + sample_period;
    current_batch_ = {};
  }

  faults_ &= ~toFlags(Fault::Disconnected);
  faults_ &= ~toFlags(Fault::OutOfRange);

  if ((faults_ & toFlags(Fault::InvalidConfig)) != 0) {
    return;
  }

  float weight_kg = config_.scale * volts + config_.offset;

  weight_kg_ = weight_kg;
  last_update_us_ = t_us;

  pushSample(t_us, units::kg_to_dkg(weight_kg));
}

void AdcLoadCell::pushSample(uint64_t t_us, int16_t weight_dkg) {
  if (current_batch_.count == 0) {
    current_batch_.t0_us = t_us;
    current_batch_.dt_us = config_.sample_period_us;
    current_batch_.count = 0;
    current_batch_.seq = batch_seq_++;
  }

  if (current_batch_.count >= BatchSampleCount) {
    flush();

    current_batch_.t0_us = t_us;
    current_batch_.dt_us = config_.sample_period_us;
    current_batch_.count = 0;
    current_batch_.seq = batch_seq_++;
  }

  current_batch_.samples[current_batch_.count] = weight_dkg;
  current_batch_.count++;

  if (current_batch_.count >= BatchSampleCount) {
    flush();
  }
}

void AdcLoadCell::flush() {
  if (current_batch_.count == 0) {
    return;
  }

  if (!batch_buffer_.push(current_batch_)) {
    faults_ |= toFlags(Fault::DataLoss);
  }

  current_batch_ = {};
}

bool AdcLoadCell::popSamples(LoadCellBatch& out) {
  return batch_buffer_.pop(out);
}

FaultFlags AdcLoadCell::validateConfig(const SensorConfig& cfg) {
  FaultFlags flags = 0;

  if (!std::isfinite(cfg.capacity_kg) ||
      !std::isfinite(cfg.scale) ||
      !std::isfinite(cfg.offset) ||
      !std::isfinite(cfg.zero_volts)) {
    flags |= toFlags(Fault::InvalidConfig);
      }

  if (cfg.capacity_kg <= 0.0f) {
    flags |= toFlags(Fault::InvalidConfig);
  }

  return flags;
}

}