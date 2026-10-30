#include "local_capacitance_sensor.hpp"

namespace sensors::capacitance {

static constexpr uint32_t StaleTimeoutUs = 100000;
static constexpr uint32_t SAMPLE_PERIOD_US = 100000;

static constexpr uint32_t MaxPollAttempts = 500;

LocalCapacitanceSensor::LocalCapacitanceSensor(
    Fdc1004& fdc1004_driver, Fdc1004Measurement measurement, Fdc1004Channel channel,
    const SensorConfig& config)
  : fdc1004_driver_(fdc1004_driver), measurement_(measurement), channel_(channel), config_(config) {
}

void LocalCapacitanceSensor::init() {
  LockGuard lock(mutex_);
  configureMeasurement();
}

float LocalCapacitanceSensor::getCapacitance() const {
  LockGuard lock(mutex_);
  return adjusted_measurement_pf_;
}

FaultFlags LocalCapacitanceSensor::getFaults() const {
  LockGuard lock(mutex_);
  return faults_;
}

SensorConfig LocalCapacitanceSensor::getConfig() const {
  LockGuard lock(mutex_);
  return config_;
}

void LocalCapacitanceSensor::update(uint64_t now_us) {
  LockGuard lock(mutex_);

  faults_ &= ~toFlags(Fault::Stale);
  faults_ &= ~toFlags(Fault::FdcFault);
  faults_ &= ~toFlags(Fault::OutOfRange);

  if (next_sample_due_us_ == 0) {
    next_sample_due_us_ = now_us;
  }

  if (now_us < next_sample_due_us_) {
    return;
  }

  next_sample_due_us_ += SAMPLE_PERIOD_US;

  // Poll the driver unitl it is ready.
  // We may also have to discard the old sample if it is using old config.
  Fdc1004Status status;
  for (uint32_t attempt = 0;; attempt++) {
    status = fdc1004_driver_.readMeasurement(measurement_, raw_measurement_pf_);

    bool retry = (status == Fdc1004Status::notReady);

    if (status == Fdc1004Status::ok && discard_next_sample_) {
      discard_next_sample_ = false;
      retry = true;
    }

    if (!retry) {
      break;
    }

    if (attempt >= MaxPollAttempts) {
      faults_ |= toFlags(Fault::FdcFault);
      return;
    }
  }

  if (status != Fdc1004Status::ok) {
    faults_ |= toFlags(Fault::FdcFault);
    return;
  }

  adjusted_measurement_pf_ = raw_measurement_pf_ + offset_pf_;

  bool rewrite_config = false;

  if (raw_measurement_pf_ > fdc1004_max_measurement_pf * 0.75f &&
      offset_pf_ < fdc1004_capdac_max_pf) {
    offset_pf_ += fdc1004_capdac_increment_pf;
    rewrite_config = true;
  } else if (raw_measurement_pf_ < fdc1004_max_measurement_pf * 0.25f &&
             offset_pf_ > 0.0f) {
    offset_pf_ -= fdc1004_capdac_increment_pf;
    rewrite_config = true;
  }

  if (rewrite_config) {
    configureMeasurement();
  }

  pushSample(now_us);

  if (last_update_us_ != 0 && (now_us - last_update_us_) > StaleTimeoutUs) {
    faults_ |= toFlags(Fault::Stale);
  }

  last_update_us_ = now_us;
}

bool LocalCapacitanceSensor::popSamples(CapacitanceBatch& out) {
  return batch_buffer_.pop(out);
}

void LocalCapacitanceSensor::configureMeasurement() {
  auto status = fdc1004_driver_.writeMeasurementConfiguration(
      measurement_,
      offset_pf_,
      channel_,
      Fdc1004Channel::capdac);

  if (status != Fdc1004Status::ok) {
    faults_ |= toFlags(Fault::FdcFault);
  }

  // The next sample we read may be from the old config.
  // Discard it to be safe.
  discard_next_sample_ = true;
}

void LocalCapacitanceSensor::pushSample(uint64_t t_us) {
  if (current_batch_.count == 0) {
    current_batch_.t0_us = t_us;
    current_batch_.dt_us = SAMPLE_PERIOD_US;
    current_batch_.seq = batch_seq_++;
  }

  const uint8_t i = current_batch_.count;

  current_batch_.samples[i].offset_cpf = pfToCpf(offset_pf_);
  current_batch_.samples[i].raw_cpf = pfToCpf(raw_measurement_pf_);
  current_batch_.samples[i].adjusted_cpf = pfToCpf(adjusted_measurement_pf_);

  current_batch_.count++;

  if (current_batch_.count >= BatchSampleCount) {
    flushBatch();
  }
}

void LocalCapacitanceSensor::flushBatch() {
  if (current_batch_.count == 0) {
    return;
  }

  if (!batch_buffer_.push(current_batch_)) {
    faults_ |= toFlags(Fault::DataLoss);
  }

  current_batch_ = {};
}

int16_t LocalCapacitanceSensor::pfToCpf(float pf) {
  const float cpf = pf * 100.0f;

  if (cpf > 32767.0f) return 32767;
  if (cpf < -32768.0f) return -32768;

  return static_cast<int16_t>(cpf);
}

}