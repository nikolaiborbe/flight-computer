#pragma once

#include <pcan_core.hpp>

#include "mutex.hpp"
#include "servo.hpp"

namespace actuators::servo {

class CanServo final : public Servo {

public:
  explicit CanServo(PCan& pcan, pcan_endpoint_id_t address);

  CommandResult beginPosition(const PositionCommand& command) override;

  WakeupResult requestWakeup(const WakeupRequest& request) override;

  ServoStatus getStatus() const override;

  const ServoModel& getModel() const override;

  ServoConfig getConfig() const override;

private:
  void callback(pcan_message_t* msg);

  PCan& pcan_;
  const pcan_endpoint_id_t address_;

  mutable Mutex mutex_;

  ServoConfig config_{};
  ServoStatus status_{};

  const ServoModel* model_ = &getServoModel(ServoModelId::Unknown);
};

}
