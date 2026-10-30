#include "can_servo_endpoint.hpp"

#include <servo_codec.hpp>

namespace actuators::servo {

CanServoEndpoint::CanServoEndpoint(PCan& pcan, pcan_endpoint_id_t addr, PwmChannel& pwm_channel,
  DigitalOut& enable_pin, ServoModelId model_id)
  : CanServoEndpoint(pcan, addr, pwm_channel, enable_pin, {.model_id = model_id}) {}

CanServoEndpoint::CanServoEndpoint(PCan& pcan, pcan_endpoint_id_t addr, PwmChannel& pwm_channel,
  DigitalOut& enable_pin, const ServoConfig& config)
  : pcan_(pcan), addr_(addr), servo_(pwm_channel, enable_pin, config) {
  pcan_.subscribe_command(0, this, &CanServoEndpoint::onCommand);
}

CommandResult CanServoEndpoint::beginPosition(const PositionCommand& command) {
  return servo_.beginPosition(command);
}

WakeupResult CanServoEndpoint::requestWakeup(const WakeupRequest& request) {
  return servo_.requestWakeup(request);
}

ServoStatus CanServoEndpoint::getStatus() const {
  return servo_.getStatus();
}

const ServoModel& CanServoEndpoint::getModel() const {
  return servo_.getModel();
}

ServoConfig CanServoEndpoint::getConfig() const {
  return servo_.getConfig();
}

void CanServoEndpoint::onAngleSample(uint64_t now_ms, float volts) {
  servo_.onMeasuredAngle(now_ms, volts);
}

void CanServoEndpoint::onCurrentSample(uint64_t now_ms, float volts) {
  servo_.onMeasuredCurrent(now_ms, volts);
}

void CanServoEndpoint::onCommand(pcan_message_t* msg) {
  const uint8_t* data = msg->data;
  const size_t len = msg->size;
  if (len < 1) return;

  const auto type = static_cast<proto::can::servo::FrameType>(data[0]);
  switch (type) {
    case proto::can::servo::FrameType::PositionCommand: {
      PositionCommand cmd{};
      if (!proto::can::servo::decode_cmd_position(data, len, cmd)) return;
      servo_.beginPosition(cmd);
    } break;

    case proto::can::servo::FrameType::WakeupCommand: {
      WakeupRequest req{};
      if (!proto::can::servo::decode_cmd_wakeup(data, len, req)) return;
      servo_.requestWakeup(req);
    } break;

    default:
      break;
  }
}

void CanServoEndpoint::update(uint64_t now_ms) {
  servo_.update(now_ms);

  const auto status = servo_.getStatus();

  constexpr uint16_t fast_period_ms = 10;
  constexpr uint16_t slow_period_ms = 200;
  constexpr uint16_t config_period_ms = 1000;

  const bool fast = status.power_state == PowerState::On || status.power_state == PowerState::Booting;
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

void CanServoEndpoint::publishStatus() const {
  auto status = servo_.getStatus();

  std::array<uint8_t, proto::can::servo::StatusLen> data{};
  proto::can::servo::encode_status(data.data(), status);

  pcan_.publish_telemetry(PCAN_PRIORITY_LOW, 0, data.data(), data.size());
}

void CanServoEndpoint::publishConfig() const {
  const auto config = servo_.getConfig();

  std::array<uint8_t, proto::can::servo::ConfigLen> data{};
  proto::can::servo::encode_config(data.data(), config);

  pcan_.publish_telemetry(PCAN_PRIORITY_LOW, 0, data.data(), data.size());
}

}
