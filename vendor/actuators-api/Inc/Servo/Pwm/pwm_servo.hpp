#pragma once

#include "digital_out.h"
#include "pwm_channel.hpp"
#include "servo.hpp"
#include "mutex.hpp"

namespace actuators::servo {

class PwmServo final : public Servo {

public:
  PwmServo(PwmChannel& pwm_channel, DigitalOut& enable_pin, ServoModelId model_id);

  PwmServo(PwmChannel& pwm_channel, DigitalOut& enable_pin, const ServoConfig& config = ServoConfig{});

  CommandResult beginPosition(const PositionCommand& command) override;

  WakeupResult requestWakeup(const WakeupRequest& request) override;

  ServoStatus getStatus() const override;

  const ServoModel& getModel() const override;

  ServoConfig getConfig() const override;

  void update(uint64_t now_ms);

  void onMeasuredAngle(uint64_t now_ms, float volts);

  void onMeasuredCurrent(uint64_t now_ms, float volts);

private:

  struct PendingCommand {
    bool valid = false;
    PositionCommand command{};
  };

  struct CurrentCommand {
    bool active = false;

    float start_angle_deg = 0.0f;
    float target_angle_deg = 0.0f;

    uint32_t delay_ms = 0;
    uint32_t duration_ms = 0;

    bool timing_initialized = false;
    bool step_applied = false;

    uint64_t accepted_at_ms = 0;
    uint64_t earliest_ready_ms = 0;
    uint64_t t_start_ms = 0;

    uint64_t t_move_end_ms = 0;
  };

  float mapVoltsToDeg(float voltage, const AngleMeasurementConfig& config) const;

  float clampToModel(float angle_deg) const;

  float angleToPulseUs(float angle_deg) const;

  uint16_t pulseUsToTicks(float pulse_us) const;

  uint32_t estimateMoveMs(float delta_deg) const;

  void applyAngleDeg(float angle_deg);

  bool handleUnknownModel();

  bool powerRequested(bool has_pending, uint64_t now_ms) const;

  void updatePower(uint64_t now_ms, bool power_requested);

  void activateCommand(const PositionCommand& cmd, uint64_t now_ms);

  void runCurrentCommand(uint64_t now_ms);

  PwmChannel& pwm_channel_;
  DigitalOut& enable_pin_;

  ServoStatus status_{};
  ServoConfig config_{};
  CurrentCommand current_command_{};
  PendingCommand pending_command_{};
  bool active_{false};

  uint32_t pending_wakeup_extra_ms_{0};

  bool had_meas_ = false;
  uint64_t last_meas_ok_ms_ = 0;

  const ServoModel& model_;

  uint64_t power_on_ms_ = 0;
  uint64_t power_hold_until_ms_ = 0;
  uint64_t last_activity_ms_ = 0;

  mutable Mutex mutex_;
};

}
