#include "gpio_solenoid.hpp"

namespace actuators::solenoid {

GpioSolenoid::GpioSolenoid(DigitalOut& enable_pin, const SolenoidConfig& config)
  : enable_pin_(enable_pin), config_(config) {
  status_.power_state = PowerState::Off;
  status_.current_ma = 0.0f;

  enable_pin_.set(false);
}

void GpioSolenoid::open() {
  LockGuard lock(mutex_, Mutex::NoWait);
  if (!lock.acquired()) return;

  const bool default_closed = config_.default_state == DefaultState::Closed;

  status_.power_state = default_closed ? PowerState::On : PowerState::Off;

  enable_pin_.set(status_.power_state == PowerState::On);
}

void GpioSolenoid::close() {
  LockGuard lock(mutex_, Mutex::NoWait);
  if (!lock.acquired()) return;

  const bool default_closed = config_.default_state == DefaultState::Closed;

  status_.power_state = default_closed ? PowerState::Off : PowerState::On;

  enable_pin_.set(status_.power_state == PowerState::On);
}

SolenoidStatus GpioSolenoid::getStatus() const {
  LockGuard lock(mutex_);

  SolenoidStatus status = status_;
  status.opening = resolveOpening(status.power_state);
  return status;
}

SolenoidConfig GpioSolenoid::getConfig() const {
  LockGuard lock(mutex_);
  return config_;
}

void GpioSolenoid::onMeasuredCurrent(const uint64_t now_ms, float volts) {
  (void)now_ms;

  LockGuard lock(mutex_);

  if (status_.power_state != PowerState::On) return;

  const auto& cs = config_.current_sense;
  if (cs.rsense_ohm <= 0.0f) return;

  const float gain = (cs.sense_ratio_k * 1000.0f) / cs.rsense_ohm;
  status_.current_ma = volts * gain;
}

Opening GpioSolenoid::resolveOpening(const PowerState power_state) const {
  if (power_state == PowerState::Unknown) return Opening::Unknown;

  const bool energized = power_state == PowerState::On;
  const bool default_closed = config_.default_state == DefaultState::Closed;

  if (!energized) return default_closed ? Opening::Closed : Opening::Open;

  return default_closed ? Opening::Open : Opening::Closed;
}

}
