#include "can_capacitance_endpoint.hpp"

#include "capacitance_codec.hpp"
#include "can_capacitance_sensor.hpp"

namespace sensors::capacitance {

CanCapacitanceEndpoint::CanCapacitanceEndpoint(
    PCan& pcan,
    pcan_endpoint_id_t addr,
    Fdc1004& fdc,
    Fdc1004Measurement measurement,
    Fdc1004Channel channel,
    const SensorConfig& sensor_cfg,
    EndpointConfig endpoint_cfg)
  : pcan_(pcan),
    addr_(addr),
    sensor_(fdc, measurement, channel, sensor_cfg),
    endpoint_cfg_(endpoint_cfg) {
  pcan_.subscribe_command(addr_.channel_id, this, &CanCapacitanceEndpoint::callback);
}

void CanCapacitanceEndpoint::init() {
  sensor_.init();
}

float CanCapacitanceEndpoint::getCapacitance() const {
  return sensor_.getCapacitance();
}

FaultFlags CanCapacitanceEndpoint::getFaults() const {
  return sensor_.getFaults();
}

SensorConfig CanCapacitanceEndpoint::getConfig() const {
  return sensor_.getConfig();
}

void CanCapacitanceEndpoint::update(uint64_t now_us) {
  sensor_.update(now_us);

  sendStatusIfNeeded(now_us);

  if (sensor_.getFaults() != 0) {
    return;
  }

  sendAvailableSamples();
}

bool CanCapacitanceEndpoint::popSamples(CapacitanceBatch& out) {
  return sensor_.popSamples(out);
}

void CanCapacitanceEndpoint::callback(pcan_message_t* msg) {
  const auto type = static_cast<FrameType>(msg->data[0]);

  switch (type) {
    default:
      break;
  }
}

void CanCapacitanceEndpoint::sendStatusIfNeeded(uint64_t now_us) {
  const bool period_elapsed = (now_us - last_status_us_) >= endpoint_cfg_.status_period_us;

  if (!period_elapsed) {
    return;
  }

  uint8_t data[proto::can::capacitance::StatusLen]{};
  const std::size_t len =
      proto::can::capacitance::encode_status(data, sensor_.getFaults(), sensor_.getConfig());

  pcan_.publish_telemetry(PCAN_PRIORITY_LOW, addr_.channel_id, data, len);

  last_status_us_ = now_us;
}

void CanCapacitanceEndpoint::sendAvailableSamples() {
  CapacitanceBatch batch{};

  while (sensor_.popSamples(batch)) {
    uint8_t data[proto::can::capacitance::SamplesMaxLen]{};
    const std::size_t len = proto::can::capacitance::encode_samples(data, batch);

    pcan_.publish_telemetry(PCAN_PRIORITY_LOW, addr_.channel_id, data, len);
  }
}

}