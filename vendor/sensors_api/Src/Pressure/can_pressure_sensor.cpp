#include "can_pressure_sensor.hpp"

#include "pressure_codec.hpp"
#include "timebase.hpp"
#include "units.hpp"

namespace sensors::pressure {

CanPressureSensor::CanPressureSensor(PCan& pcan, pcan_endpoint_id_t address)
  : pcan_(pcan), address_(address) {
  pcan_.subscribe_telemetry(address_, this, &CanPressureSensor::callback);
}

float CanPressureSensor::getPressure() const {
  LockGuard lock(mutex_);
  return pressure_bar_;
}

FaultFlags CanPressureSensor::getFaults() const {
  LockGuard lock(mutex_);
  return faults_;
}

void CanPressureSensor::setSamplePeriod(const uint32_t sample_period_us) {
  SensorConfig requested{};

  {
    LockGuard lock(mutex_);
    requested = sensor_config_;
  }

  requested.sample_period_us = sample_period_us;

  uint8_t data[proto::can::pressure::SetConfigLen]{};
  size_t len = proto::can::pressure::encode_set_config(data, requested);

  pcan_.publish_command(PCAN_PRIORITY_HIGH, address_, data, len);
}

SensorConfig CanPressureSensor::getConfig() const {
  LockGuard lock(mutex_);
  return sensor_config_;
}

void CanPressureSensor::update(uint64_t now_us) {
  LockGuard lock(mutex_);

  if (last_rx_us_ == 0 || (now_us - last_rx_us_) > CommsTimeoutUs) {
    faults_ |= toFlags(Fault::CommFault);
  } else {
    faults_ &= ~toFlags(Fault::CommFault);
  }
}

bool CanPressureSensor::popSamples(PressureBatch& out) {
  return batch_buffer_.pop(out);
}

bool CanPressureSensor::popAbortSamples(PressureBatch& out) {
  return abort_buffer_.pop(out);
}

void CanPressureSensor::callback(pcan_message_t* msg) {
  const auto type = static_cast<FrameType>(msg->data[0]);

  switch (type) {
    case FrameType::Status:
      handleStatus(msg->data, msg->size);
      break;

    case FrameType::Samples:
      handleSamples(msg->data, msg->size);
      break;

    default:
      break;
  }
}

void CanPressureSensor::handleStatus(const uint8_t* payload, std::size_t len) {
  StatusFrame frame{};

  if (!proto::can::pressure::decode_status(payload, len, frame)) {
    return;
  }

  LockGuard lock(mutex_);
  sensor_config_ = frame.sensor_config;
  faults_ = frame.faults;
  last_rx_us_ = timebase::micros();
  faults_ &= ~toFlags(Fault::CommFault);
}

void CanPressureSensor::handleSamples(const uint8_t *payload, std::size_t len) {
  PressureBatch batch{};

  if (!proto::can::pressure::decode_samples(payload, len, batch)) {
    return;
  }

  {
    LockGuard lock(mutex_);

    if (batch_seq_valid_) {
      const uint8_t expected = static_cast<uint8_t>(last_batch_seq_ + 1u);
      if (batch.seq != expected) {
        ++rx_drops_;
        faults_ |= toFlags(Fault::DataLoss);
      } else {
        faults_ &= ~toFlags(Fault::DataLoss);
      }
    }

    batch_seq_valid_ = true;
    last_batch_seq_ = batch.seq;
    last_rx_us_ = timebase::micros();

    if (batch.count > 0) {
      pressure_bar_ = units::cbarToBar(batch.samples[batch.count - 1]);
    }

    faults_ &= ~toFlags(Fault::CommFault);
  }

  bool data_loss = false;

  if (!batch_buffer_.push(batch)) {
    data_loss = true;
  }

  if (!abort_buffer_.push(batch)) {
    data_loss = true;
  }

  if (data_loss) {
    LockGuard lock(mutex_);
    ++rx_drops_;
    faults_ |= toFlags(Fault::DataLoss);
  }
}

}
