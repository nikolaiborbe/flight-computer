#pragma once

#include <pcan_core.hpp>
#include <pwm_servo.hpp>

namespace actuators::servo {

class CanServoEndpoint : public Servo {
public:

  CanServoEndpoint(PCan& pcan, pcan_endpoint_id_t addr, PwmChannel& pwm_channel,
    DigitalOut& enable_pin, ServoModelId model_id);

  CanServoEndpoint(PCan& pcan, pcan_endpoint_id_t addr, PwmChannel& pwm_channel,
    DigitalOut& enable_pin, const ServoConfig& config = {});

  CommandResult beginPosition(const PositionCommand& command) override;

  WakeupResult requestWakeup(const WakeupRequest& request) override;

  ServoStatus getStatus() const override;

  const ServoModel& getModel() const override;

  ServoConfig getConfig() const override;

  void update(uint64_t now_ms);

  void onAngleSample(uint64_t now_ms, float volts);

  void onCurrentSample(uint64_t now_ms, float volts);

private:
  void onCommand(pcan_message_t* msg);

  void publishStatus() const;

  void publishConfig() const;

  PCan& pcan_;
  pcan_endpoint_id_t addr_;
  PwmServo servo_;

  uint64_t next_status_ms_ = 0;
  uint64_t next_config_ms_ = 0;
};

}
