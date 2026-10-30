#include "adc_pressure_sensor.hpp"
#include "units.hpp"

#include <cmath>

namespace sensors::pressure {

static constexpr float PressureEps = 1e-3f;
static constexpr uint32_t StaleTimeoutUs = 100000;

AdcPressureSensor::AdcPressureSensor(const SensorConfig& config)
  : config_(config) {}

float AdcPressureSensor::getPressure() const {
  LockGuard lock(mutex_);
  return pressure_bar_;
}

FaultFlags AdcPressureSensor::getFaults() const {
  LockGuard lock(mutex_);
  return faults_;
}

void AdcPressureSensor::setSamplePeriod(uint32_t sample_period_us) {
  LockGuard lock(mutex_);
  config_.sample_period_us = sample_period_us;
  next_sample_due_us_ = 0;
  current_batch_ = {};
}

void AdcPressureSensor::setSensorConfig(const SensorConfig& config) {
  LockGuard lock(mutex_);
  config_ = config;
  next_sample_due_us_ = 0;
  current_batch_ = {};
}

SensorConfig AdcPressureSensor::getConfig() const {
  LockGuard lock(mutex_);
  return config_;
}

void AdcPressureSensor::update(uint64_t now_us) {
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

void AdcPressureSensor::onSample(const uint64_t t_us, const float volts) {
  adc_buffer_.push({t_us, volts});
}

void AdcPressureSensor::processSample(uint64_t t_us, float volts) {
  // Assuming lock is held
  if (last_sample_us_ != 0 && t_us < last_sample_us_) {
    next_sample_due_us_ = 0;
    current_batch_ = {};
  }

  last_sample_us_ = t_us;

  if (next_sample_due_us_ == 0) next_sample_due_us_ = t_us;
  if (t_us < next_sample_due_us_) {
    return;
  }

  uint32_t sample_period = config_.sample_period_us;
  if (sample_period == 0) return;

  next_sample_due_us_ += sample_period;

  if (t_us > next_sample_due_us_ + sample_period) {
    next_sample_due_us_ = t_us + sample_period;
    current_batch_ = {};
  }

  if ((faults_ & toFlags(Fault::InvalidConfig)) != 0) {
    return;
  }

  faults_ &= ~toFlags(Fault::Disconnected);
  faults_ &= ~toFlags(Fault::OutOfRange);

  float p = 0.0f;

  if (config_.output_type == OutputType::Voltage) {
    if (volts < (config_.v_min - config_.margin) || volts > (config_.v_max + config_.margin)) {
      faults_ |= toFlags(Fault::OutOfRange);
    }

    p = mapLinear(volts, config_.v_min, config_.v_max, config_.p_min_bar, config_.p_max_bar);
  } else {
    const float i_ma = (volts / config_.shunt_ohms) * 1000.0f;
    if (i_ma < config_.disconnect_threshold) {
      faults_ |= toFlags(Fault::Disconnected);
    } else if (i_ma < (config_.i_min_ma - config_.margin) || i_ma > (config_.i_max_ma + config_.margin)) {
      faults_ |= toFlags(Fault::OutOfRange);
    }

    p = mapLinear(i_ma, config_.i_min_ma, config_.i_max_ma, config_.p_min_bar, config_.p_max_bar);
  }

  if (!std::isfinite(p) || p < (config_.p_min_bar - PressureEps) || p > (config_.p_max_bar + PressureEps)) {
    faults_ |= toFlags(Fault::OutOfRange);
  }

  pressure_bar_ = p;
  last_update_us_ = t_us;

  pushSample(t_us, units::barToCbar(p));
}

void AdcPressureSensor::pushSample(uint64_t t_us, int16_t pressure_cbar) {
  // Assuming lock is held.
  if (current_batch_.count == 0) {
    current_batch_.t0_us = t_us;
    current_batch_.dt_us = config_.sample_period_us;
    current_batch_.count = 0;
  }

  current_batch_.samples[current_batch_.count] = pressure_cbar;
  current_batch_.count++;

  if (current_batch_.count >= BatchSampleCount) {
    flushBatch();
  }
}

void AdcPressureSensor::flushBatch() {
  // Assuming lock is held.
  if (current_batch_.count == 0) {
    return;
  }

  current_batch_.seq = batch_seq_++;

  if (!batch_buffer_.push(current_batch_)) {
    faults_ |= toFlags(Fault::DataLoss);
  }

  current_batch_ = {};
}

bool AdcPressureSensor::popSamples(PressureBatch& out) {
  return batch_buffer_.pop(out);
}

float AdcPressureSensor::mapLinear(float x, float x0, float x1, float y0, float y1) {
  if (x1 == x0) return 0.0f;
  const float t = (x - x0) / (x1 - x0);
  return y0 + t * (y1 - y0);
}

FaultFlags AdcPressureSensor::validateConfig(const SensorConfig& cfg) {
  FaultFlags flags = 0;

  if (cfg.p_max_bar <= cfg.p_min_bar) {
    flags |= toFlags(Fault::InvalidConfig);
  }

  if (cfg.margin < 0.0f) {
    flags |= toFlags(Fault::InvalidConfig);
  }

  if (cfg.output_type == OutputType::Voltage) {
    if (cfg.v_max <= cfg.v_min) {
      flags |= toFlags(Fault::InvalidConfig);
    }
  } else {
    if (cfg.i_max_ma <= cfg.i_min_ma) {
      flags |= toFlags(Fault::InvalidConfig);
    }
    if (cfg.shunt_ohms <= 0.0f) {
      flags |= toFlags(Fault::InvalidConfig);
    }
  }

  return flags;
}

}
