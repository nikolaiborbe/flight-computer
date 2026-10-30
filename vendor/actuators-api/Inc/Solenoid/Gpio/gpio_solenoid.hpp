#pragma once

#include "digital_out.h"
#include "solenoid.hpp"
#include "mutex.hpp"

namespace actuators::solenoid {

class GpioSolenoid final : public Solenoid {

public:
  GpioSolenoid(DigitalOut& enable_pin, const SolenoidConfig& config = SolenoidConfig{});

  void open() override;

  void close() override;

  SolenoidStatus getStatus() const override;

  SolenoidConfig getConfig() const override;

  void onMeasuredCurrent(uint64_t now_ms, float volts);

private:
  Opening resolveOpening(PowerState power_state) const;

  DigitalOut& enable_pin_;

  SolenoidStatus status_{};
  SolenoidConfig config_{};

  mutable Mutex mutex_;
};

}
