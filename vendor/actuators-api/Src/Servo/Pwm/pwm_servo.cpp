#include "pwm_servo.hpp"

#include <cmath>
#include <algorithm>

namespace actuators::servo {

static ServoConfig makeConfig(ServoModelId model_id) {
  ServoConfig cfg{};
  cfg.model_id = model_id;
  return cfg;
}

PwmServo::PwmServo(PwmChannel& pwm_channel, DigitalOut& enable_pin, const ServoModelId model_id)
  : PwmServo(pwm_channel, enable_pin, makeConfig(model_id)) {}

PwmServo::PwmServo(PwmChannel& pwm_channel, DigitalOut& enable_pin, const ServoConfig& config)
  : pwm_channel_(pwm_channel), enable_pin_(enable_pin),
  config_(config), model_(getServoModel(config.model_id)) {
  status_.commanded_angle_deg = 0.0f;
  status_.target_angle_deg = 0.0f;
  status_.measured_angle_deg = 0.0f;
  status_.power_state = PowerState::Off;

  enable_pin_.set(false);

  power_on_ms_ = 0;
  last_activity_ms_ = 0;

  const bool volts_unset = (config_.angle_measurement.v_at_min == 0.0f &&
                          config_.angle_measurement.v_at_max == 0.0f);

  if (volts_unset) {
    config_.angle_measurement.v_at_min = model_.angle_calibration.v_min_deg;
    config_.angle_measurement.v_at_max = model_.angle_calibration.v_max_deg;
  }
}

CommandResult PwmServo::beginPosition(const PositionCommand& command) {
  LockGuard lock(mutex_, Mutex::NoWait);

  if (!lock.acquired()) {
    return CommandResult::RejectedBusy;
  }

  if (model_.id == ServoModelId::Unknown) {
    return CommandResult::RejectedInvalid;
  }

  const bool busy = pending_command_.valid || active_;
  if (command.start_policy == StartPolicy::NoInterrupt && busy) {
    return CommandResult::RejectedBusy;
  }

  pending_command_.command = command;
  pending_command_.valid = true;

  return busy ? CommandResult::PreemptedPrevious : CommandResult::Accepted;
}

WakeupResult PwmServo::requestWakeup(const WakeupRequest& request) {
  LockGuard lock(mutex_, Mutex::NoWait);

  if (!lock.acquired()) {
    return WakeupResult::RejectedBusy;
  }

  if (model_.id == ServoModelId::Unknown) return WakeupResult::RejectedInvalid;

  uint32_t requested = request.keep_alive_ms;
  if (requested == 0u) requested = 1u; // treat 0 as "wake-only"

  if (requested > pending_wakeup_extra_ms_) {
    pending_wakeup_extra_ms_ = requested;
  }

  return WakeupResult::Ok;
}

ServoStatus PwmServo::getStatus() const {
  LockGuard lock(mutex_);
  return status_;
}

const ServoModel& PwmServo::getModel() const {
  return model_;
}

ServoConfig PwmServo::getConfig() const {
  LockGuard lock(mutex_);
  return config_;
}

void PwmServo::update(const uint64_t now_ms) {
  LockGuard lock(mutex_);

  if (handleUnknownModel()) {
    return;
  }

  PendingCommand pc{};
  if (pending_command_.valid) {
    pc = pending_command_;
    pending_command_ = PendingCommand{};
  }

  const bool has_pending = pc.valid;

  const uint32_t extra_ms_u32 = pending_wakeup_extra_ms_;
  pending_wakeup_extra_ms_ = 0;

  if (extra_ms_u32 != 0u) {
    constexpr uint64_t margin_ms = 10u;
    const uint64_t boot_ms  = model_.boot_time_ms;
    const uint64_t extra_ms = extra_ms_u32;

    const bool needs_boot =
        (status_.power_state == PowerState::Off ||
         status_.power_state == PowerState::Unknown ||
         status_.power_state == PowerState::Booting);

    const uint64_t requested = (needs_boot ? (boot_ms + margin_ms) : 0u) + extra_ms;
    const uint64_t new_until = now_ms + requested;

    if (new_until > power_hold_until_ms_) power_hold_until_ms_ = new_until;
  }

  const bool power_requested = powerRequested(has_pending, now_ms);
  updatePower(now_ms, power_requested);

  // If we consumed a new command, activate it (preempting any current one)
  if (pc.valid) {
    activateCommand(pc.command, now_ms);
  }

  // Run motion profile (or do nothing if no active command)
  runCurrentCommand(now_ms);
}

bool PwmServo::handleUnknownModel() {
  if (model_.id != ServoModelId::Unknown) return false;

  enable_pin_.set(false);

  status_.power_state = PowerState::Unknown;
  return true;
}

bool PwmServo::powerRequested(const bool has_pending, const uint64_t now_ms) const {
  const bool lease_active = (now_ms < power_hold_until_ms_);
  return (config_.power_mode == PowerMode::KeepOn) ||
         active_ ||
         has_pending ||
         lease_active;
}

void PwmServo::updatePower(const uint64_t now_ms, const bool power_requested) {
  // Request power: Off -> Booting
  if (power_requested && (status_.power_state == PowerState::Off || status_.power_state == PowerState::Unknown)) {
    enable_pin_.set(true);
    pwm_channel_.start();
    status_.power_state = PowerState::Booting;
    power_on_ms_ = now_ms;
  }

  // Booting -> On after boot time
  if (status_.power_state == PowerState::Booting) {
    if ((now_ms - power_on_ms_) >= model_.boot_time_ms) {
      status_.power_state = PowerState::On;
    }
  }

  // Auto-off: only when not requested and idle timeout elapsed
  if (config_.power_mode == PowerMode::AutoOff && !power_requested &&
      (status_.power_state == PowerState::On || status_.power_state == PowerState::Booting)) {

    const uint64_t idle_for = (now_ms >= last_activity_ms_) ? (now_ms - last_activity_ms_) : 0u;

    // 0 => never auto-off
    if (config_.idle_timeout_ms != 0u) {
      const uint64_t timeout_ms = config_.idle_timeout_ms;
      if (idle_for >= timeout_ms) {
        enable_pin_.set(false);
        pwm_channel_.stop();
        status_.power_state = PowerState::Off;
      }
    }
  }
}

void PwmServo::activateCommand(const PositionCommand& cmd, const uint64_t now_ms) {
  current_command_ = CurrentCommand{};
  current_command_.active = true;

  current_command_.delay_ms = cmd.delay_ms;
  current_command_.duration_ms = cmd.duration_ms;
  current_command_.timing_initialized = false;
  current_command_.step_applied = false;

  active_ = true;

  current_command_.accepted_at_ms = now_ms;

  // Boot time acts as extra delay before command can start
  current_command_.earliest_ready_ms = power_on_ms_ + static_cast<uint64_t>(model_.boot_time_ms);

  current_command_.start_angle_deg = status_.commanded_angle_deg;

  float target = cmd.angle_deg;
  if (cmd.position_mode == PositionMode::Relative) {
    target = status_.commanded_angle_deg + cmd.angle_deg;
  }

  current_command_.target_angle_deg = clampToModel(target);

  status_.target_angle_deg = current_command_.target_angle_deg;
  last_activity_ms_ = now_ms;
}

void PwmServo::runCurrentCommand(const uint64_t now_ms) {
  if (!current_command_.active) {
    return;
  }

  // If power is off, cannot move (keep command active as before)
  if (status_.power_state == PowerState::Off) {
    return;
  }

  // Initialize timing
  if (!current_command_.timing_initialized) {
    const uint64_t base = (now_ms > current_command_.earliest_ready_ms)
                            ? now_ms
                            : current_command_.earliest_ready_ms;

    current_command_.t_start_ms = base + current_command_.delay_ms;

    if (current_command_.duration_ms == 0u) {
      const float delta = current_command_.target_angle_deg - current_command_.start_angle_deg;

      uint32_t phys_ms = estimateMoveMs(delta);
      if (phys_ms == 0u && delta != 0.0f) {
        phys_ms = 1u;
      }

      current_command_.t_move_end_ms = current_command_.t_start_ms + phys_ms;
    } else {
      // Ramp duration defines the "end" of the profile and thus completion.
      current_command_.t_move_end_ms = current_command_.t_start_ms + current_command_.duration_ms;
    }

    current_command_.timing_initialized = true;
    current_command_.step_applied = false;
  }

  // Pre-start: do nothing while waiting for boot + delay
  if (now_ms < current_command_.t_start_ms) {
    return;
  }

  // Execute motion
  if (current_command_.duration_ms == 0u) {
    if (!current_command_.step_applied) {
      applyAngleDeg(current_command_.target_angle_deg);
      status_.commanded_angle_deg = current_command_.target_angle_deg;
      current_command_.step_applied = true;
      last_activity_ms_ = now_ms;
    }
  } else {
    const uint64_t span = current_command_.t_move_end_ms - current_command_.t_start_ms;
    const uint64_t t = now_ms - current_command_.t_start_ms;
    const float alpha = (span == 0u) ? 1.0f : static_cast<float>(t) / static_cast<float>(span);

    float new_angle =
      current_command_.start_angle_deg +
      alpha * (current_command_.target_angle_deg - current_command_.start_angle_deg);

    new_angle = clampToModel(new_angle);
    applyAngleDeg(new_angle);

    status_.commanded_angle_deg = new_angle;
    last_activity_ms_ = now_ms;
  }

  // Completion
  if (now_ms >= current_command_.t_move_end_ms) {
    applyAngleDeg(current_command_.target_angle_deg);
    status_.commanded_angle_deg = current_command_.target_angle_deg;

    current_command_ = CurrentCommand{};
    active_ = false;
    last_activity_ms_ = now_ms;
  }
}

void PwmServo::onMeasuredAngle(const uint64_t now_ms, float volts) {
  LockGuard lock(mutex_);

  if (status_.power_state != PowerState::On) {
    return; // keeps last known measurement
  }

  const auto& config = config_.angle_measurement;

  if (config.v_at_min == config.v_at_max) return;

  float deg = mapVoltsToDeg(volts, config);
  deg = clampToModel(deg);

  status_.measured_angle_deg = deg;
  last_meas_ok_ms_ = now_ms;
  had_meas_ = true;
}

void PwmServo::onMeasuredCurrent(const uint64_t now_ms, float volts) {
  LockGuard lock(mutex_);

  if (status_.power_state != PowerState::On &&
      status_.power_state != PowerState::Booting) {
    return;
  }

  const auto& cs = config_.current_sense;
  if (cs.rsense_ohm <= 0.0f) return;

  const float gain = (cs.sense_ratio_k * 1000.0f) / cs.rsense_ohm;
  const float ma = volts * gain;

  status_.current_ma = ma;
}

float PwmServo::mapVoltsToDeg(float v, const AngleMeasurementConfig& config) const {
  const float a = config.v_at_min;
  const float b = config.v_at_max;
  const float denom = (b - a);

  if (denom == 0.0f) return 0.0f;

  float x = (v - a) / denom;
  x = std::clamp(x, 0.0f, 1.0f);

  return model_.angle_limits.min_deg + x * (model_.angle_limits.max_deg - model_.angle_limits.min_deg);
}


float PwmServo::clampToModel(const float angle_deg) const {
  return std::clamp(angle_deg, model_.angle_limits.min_deg, model_.angle_limits.max_deg);
}

float PwmServo::angleToPulseUs(const float angle_deg) const {
  const float min_deg = model_.angle_limits.min_deg;
  const float max_deg = model_.angle_limits.max_deg;
  const float span_deg = max_deg - min_deg;

  const float min_us = model_.pulse_range.min_us;
  const float max_us = model_.pulse_range.max_us;
  const float span_us = max_us - min_us;

  if (span_deg <= 0.0f) return min_us;

  float norm = (angle_deg - min_deg) / span_deg;
  norm = std::clamp(norm, 0.0f, 1.0f);

  return min_us + norm * span_us;
}

uint16_t PwmServo::pulseUsToTicks(float pulse_us) const {
  if (pulse_us < 0.0f) pulse_us = 0.0f;

  const float ticks_per_us = pwm_channel_.ticksPerUs();

  const float ticks_f = pulse_us * ticks_per_us;
  const float rounded = std::round(ticks_f);

  if (rounded < 0.0f) return 0;
  if (rounded > 65535.0f) return 65535;
  return static_cast<uint16_t>(rounded);
}

uint32_t PwmServo::estimateMoveMs(const float delta_deg) const {
  if (model_.sec_per_60_deg <= 0.0f) {
    return 0u;
  }

  const float abs_deg = std::fabs(delta_deg);
  const float sec = model_.sec_per_60_deg * (abs_deg / 60.0f);
  const float ms = sec * 1000.0f;

  if (ms < 0.0f) return 0u;

  const float rounded = std::ceil(ms);
  if (rounded > 4294967295.0f) return 0xFFFFFFFFu;
  return static_cast<uint32_t>(rounded);
}

void PwmServo::applyAngleDeg(const float angle_deg) {
  float pulse_us = angleToPulseUs(angle_deg);
  const uint16_t ticks = pulseUsToTicks(pulse_us);
  pwm_channel_.setCompare(ticks);
}

}
