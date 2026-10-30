#include "can_solenoid.hpp"

#include "solenoid_codec.hpp"

namespace actuators::solenoid {

CanSolenoid::CanSolenoid(PCan& pcan, pcan_endpoint_id_t address)
  : pcan_(pcan), address_(address) {
  pcan_.subscribe_telemetry(address, this, &CanSolenoid::callback);
}

void CanSolenoid::open() {
  std::array<uint8_t, proto::can::solenoid::OpenCmdLen> data{};
  proto::can::solenoid::encode_cmd_open(data.data());

  (void)pcan_.publish_command(PCAN_PRIORITY_HIGH, address_, data.data(), data.size());
}

void CanSolenoid::close() {
  std::array<uint8_t, proto::can::solenoid::CloseCmdLen> data{};
  proto::can::solenoid::encode_cmd_close(data.data());

  (void)pcan_.publish_command(PCAN_PRIORITY_HIGH, address_, data.data(), data.size());
}

SolenoidStatus CanSolenoid::getStatus() const {
  LockGuard lock(mutex_);
  return status_;
}

SolenoidConfig CanSolenoid::getConfig() const {
  LockGuard lock(mutex_);
  return config_;
}

void CanSolenoid::callback(pcan_message_t* msg) {
  if (msg->size < 1) return;

  const auto type = static_cast<proto::can::solenoid::FrameType>(msg->data[0]);
  switch (type) {
    case proto::can::solenoid::FrameType::Status: {
      SolenoidStatus status{};
      if (!proto::can::solenoid::decode_status(msg->data, msg->size, status)) return;
      LockGuard lock(mutex_);
      status_ = status;
    } break;

    case proto::can::solenoid::FrameType::Config: {
      SolenoidConfig config{};
      if (!proto::can::solenoid::decode_config(msg->data, msg->size, config)) return;
      LockGuard lock(mutex_);
      config_ = config;
    } break;

    default:
      break;
  }
}

}
