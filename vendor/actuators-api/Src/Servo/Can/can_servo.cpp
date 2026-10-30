#include "can_servo.hpp"

#include "servo_codec.hpp"

namespace actuators::servo {

CanServo::CanServo(PCan& pcan, pcan_endpoint_id_t address)
  : pcan_(pcan), address_(address) {
  pcan_.subscribe_telemetry(address, this, &CanServo::callback);
}

CommandResult CanServo::beginPosition(const PositionCommand& command) {
  std::array<uint8_t, proto::can::servo::PosCmdLen> data{};
  proto::can::servo::encode_cmd_position(data.data(), command);

  const int r = pcan_.publish_command(PCAN_PRIORITY_HIGH, address_, data.data(), data.size());

  return r == 0 ? CommandResult::Accepted : CommandResult::RejectedBusy;
}

WakeupResult CanServo::requestWakeup(const WakeupRequest& request) {
  std::array<uint8_t, proto::can::servo::WakeCmdLen> data{};
  proto::can::servo::encode_cmd_wakeup(data.data(), request);

  const int r = pcan_.publish_command(PCAN_PRIORITY_HIGH, address_, data.data(), data.size());

  return r == 0 ? WakeupResult::Ok : WakeupResult::RejectedBusy;
}

ServoStatus CanServo::getStatus() const {
  LockGuard lock(mutex_);
  return status_;
}

const ServoModel& CanServo::getModel() const {
  return *model_;
}

ServoConfig CanServo::getConfig() const {
  LockGuard lock(mutex_);
  return config_;
}

void CanServo::callback(pcan_message_t* msg) {
  if (msg->size < 1) return;

  const auto type = static_cast<proto::can::servo::FrameType>(msg->data[0]);
  switch (type) {
    case proto::can::servo::FrameType::Status: {
      ServoStatus status{};
      if (!proto::can::servo::decode_status(msg->data, msg->size, status)) return;
      LockGuard lock(mutex_);
      status_ = status;
    } break;

    case proto::can::servo::FrameType::Config: {
      ServoConfig config{};
      if (!proto::can::servo::decode_config(msg->data, msg->size, config)) return;
      LockGuard lock(mutex_);
      config_ = config;
      model_ = &getServoModel(config_.model_id);
    } break;

    default:
      break;
  }
}

}
