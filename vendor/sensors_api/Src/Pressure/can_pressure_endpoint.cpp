#include "can_pressure_endpoint.hpp"

#include <cstddef>
#include <cstring>

#include "pressure_codec.hpp"

namespace sensors::pressure {

CanPressureEndpoint::CanPressureEndpoint(PCan& pcan,
                                       pcan_endpoint_id_t addr,
                                       const SensorConfig& sensor_cfg,
                                       EndpointConfig adapter_cfg)
    : pcan_(pcan),
      addr_(addr),
      sensor_(sensor_cfg),
      adapter_cfg_(adapter_cfg) {
  pcan_.subscribe_command(addr_.channel_id, this, &CanPressureEndpoint::callback);
}

void CanPressureEndpoint::onSample(uint64_t now_us, float volts) {
  sensor_.onSample(now_us, volts);
}

void CanPressureEndpoint::update(uint64_t now_us) {
  sensor_.update(now_us);

  sendStatusIfNeeded(now_us);

  sendAvailableSamples();
}

float CanPressureEndpoint::getPressure() const {
  return sensor_.getPressure();
}

FaultFlags CanPressureEndpoint::getFaults() const {
  return sensor_.getFaults();
}

void CanPressureEndpoint::setSamplePeriod(uint32_t sample_period_us) {
  sensor_.setSamplePeriod(sample_period_us);
}

SensorConfig CanPressureEndpoint::getConfig() const {
  return sensor_.getConfig();
}

bool CanPressureEndpoint::popSamples(PressureBatch& out) {
  return sensor_.popSamples(out);
}

void CanPressureEndpoint::callback(pcan_message_t* msg) {
  const auto type = static_cast<FrameType>(msg->data[0]);

  switch (type) {
    case FrameType::SetConfig:
      handleSetConfig(msg->data, msg->size);
      break;

    default:
      break;
  }
}

void CanPressureEndpoint::handleSetConfig(const uint8_t *payload, std::size_t len) {
  SensorConfig cfg{};

  if (!proto::can::pressure::decode_set_config(payload, len, cfg)) {
    return;
  }

  sensor_.setSensorConfig(cfg);
}

void CanPressureEndpoint::sendStatusIfNeeded(uint64_t now_us) {
  const bool period_elapsed = (now_us - last_status_us_) >= adapter_cfg_.status_period_us;

  if (!period_elapsed) {
    return;
  }

  uint8_t data[proto::can::pressure::StatusLen]{};
  size_t len = proto::can::pressure::encode_status(data, sensor_.getFaults(), sensor_.getConfig());
  pcan_.publish_telemetry(PCAN_PRIORITY_LOW, addr_.channel_id, data, len);

  last_status_us_ = now_us;
}

void CanPressureEndpoint::sendAvailableSamples() {
  PressureBatch batch{};

  while (sensor_.popSamples(batch)) {
    uint8_t data[proto::can::pressure::SamplesMaxLen]{};
    const std::size_t len = proto::can::pressure::encode_samples(data, batch);

    pcan_.publish_telemetry(PCAN_PRIORITY_LOW, addr_.channel_id, data, len);
  }
}

}
