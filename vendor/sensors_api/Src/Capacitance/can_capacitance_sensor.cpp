#include "can_capacitance_sensor.hpp"

#include "capacitance_codec.hpp"
#include "timebase.hpp"

namespace sensors::capacitance {

CanCapacitanceSensor::CanCapacitanceSensor(PCan& pcan, pcan_endpoint_id_t address)
  : pcan_(pcan), address_(address) {
  pcan_.subscribe_telemetry(address_, this, &CanCapacitanceSensor::callback);
}

float CanCapacitanceSensor::getCapacitance() const {
  LockGuard lock(mutex_);
  return adjusted_measurement_pf_;
}

FaultFlags CanCapacitanceSensor::getFaults() const {
  LockGuard lock(mutex_);
  return faults_;
}

SensorConfig CanCapacitanceSensor::getConfig() const {
  LockGuard lock(mutex_);
  return sensor_config_;
}

void CanCapacitanceSensor::update(uint64_t now_us) {
  LockGuard lock(mutex_);

  if (last_rx_us_ == 0 || (now_us - last_rx_us_) > CommsTimeoutUs) {
    faults_ |= toFlags(Fault::CommFault);
  } else {
    faults_ &= ~toFlags(Fault::CommFault);
  }
}

bool CanCapacitanceSensor::popSamples(CapacitanceBatch& out) {
  return batch_buffer_.pop(out);
}

void CanCapacitanceSensor::callback(pcan_message_t* msg) {
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

void CanCapacitanceSensor::handleStatus(const uint8_t* payload, std::size_t len) {
  StatusFrame frame{};

  if (!proto::can::capacitance::decode_status(payload, len, frame)) {
    return;
  }

  LockGuard lock(mutex_);
  sensor_config_ = frame.sensor_config;
  faults_ = frame.faults;
  last_rx_us_ = timebase::micros();
  faults_ &= ~toFlags(Fault::CommFault);
}

void CanCapacitanceSensor::handleSamples(const uint8_t* payload, std::size_t len) {
  CapacitanceBatch batch{};

  if (!proto::can::capacitance::decode_samples(payload, len, batch)) {
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
      const auto& latest = batch.samples[batch.count - 1];

      offset_pf_ = cpfToPf(latest.offset_cpf);
      raw_measurement_pf_ = cpfToPf(latest.raw_cpf);
      adjusted_measurement_pf_ = cpfToPf(latest.adjusted_cpf);
    }

    faults_ &= ~toFlags(Fault::CommFault);
  }

  if (!batch_buffer_.push(batch)) {
    LockGuard lock(mutex_);
    ++rx_drops_;
    faults_ |= toFlags(Fault::DataLoss);
  }
}

float CanCapacitanceSensor::cpfToPf(int16_t cpf) {
  return static_cast<float>(cpf) / 100.0f;
}

}