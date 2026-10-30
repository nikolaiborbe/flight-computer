#include "can_solenoid_endpoint.hpp"

#include <solenoid_codec.hpp>

namespace actuators::solenoid {

CanSolenoidEndpoint::CanSolenoidEndpoint(PCan& pcan, pcan_node_id_t addr, DigitalOut& enable_pin,
  const SolenoidConfig& config)
  : pcan_(pcan), addr_(addr), solenoid_(enable_pin, config) {
  pcan_.subscribe_command(0, this, &CanSolenoidEndpoint::onCommand);
}

void CanSolenoidEndpoint::open() {
  solenoid_.open();
}

void CanSolenoidEndpoint::close() {
  solenoid_.close();
}

SolenoidStatus CanSolenoidEndpoint::getStatus() const {
  return solenoid_.getStatus();
}

SolenoidConfig CanSolenoidEndpoint::getConfig() const {
  return solenoid_.getConfig();
}

void CanSolenoidEndpoint::onCurrentSample(uint64_t now_ms, float volts) {
  solenoid_.onMeasuredCurrent(now_ms, volts);
}

void CanSolenoidEndpoint::onCommand(pcan_message_t* msg) {
  const uint8_t* data = msg->data;
  const size_t len = msg->size;
  if (len < 1) return;

  const auto type = static_cast<proto::can::solenoid::FrameType>(data[0]);
  switch (type) {
    case proto::can::solenoid::FrameType::OpenCommand:
      if (!proto::can::solenoid::decode_cmd_open(data, len)) return;
      solenoid_.open();
      break;

    case proto::can::solenoid::FrameType::CloseCommand:
      if (!proto::can::solenoid::decode_cmd_close(data, len)) return;
      solenoid_.close();
      break;

    default:
      break;
  }
}

void CanSolenoidEndpoint::update(uint64_t now_ms) {
  const auto status = solenoid_.getStatus();

  constexpr uint16_t fast_period_ms = 10;
  constexpr uint16_t slow_period_ms = 200;
  constexpr uint16_t config_period_ms = 1000;

  const bool fast = status.power_state == PowerState::On;
  const uint16_t status_period = fast ? fast_period_ms : slow_period_ms;

  if (now_ms >= next_status_ms_) {
    publishStatus();
    next_status_ms_ = now_ms + status_period;
  }

  if (now_ms >= next_config_ms_) {
    publishConfig();
    next_config_ms_ = now_ms + config_period_ms;
  }
}

void CanSolenoidEndpoint::publishStatus() const {
  auto status = solenoid_.getStatus();

  std::array<uint8_t, proto::can::solenoid::StatusLen> data{};
  proto::can::solenoid::encode_status(data.data(), status);

  pcan_.publish_telemetry(PCAN_PRIORITY_LOW, 0, data.data(), data.size());
}

void CanSolenoidEndpoint::publishConfig() const {
  const auto config = solenoid_.getConfig();

  std::array<uint8_t, proto::can::solenoid::ConfigLen> data{};
  proto::can::solenoid::encode_config(data.data(), config);

  pcan_.publish_telemetry(PCAN_PRIORITY_LOW, 0, data.data(), data.size());
}

}
