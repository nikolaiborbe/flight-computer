#pragma once

#include <pcan_core.hpp>

#include "mutex.hpp"
#include "solenoid.hpp"

namespace actuators::solenoid {

class CanSolenoid final : public Solenoid {

public:
  explicit CanSolenoid(PCan& pcan, pcan_endpoint_id_t address);

  void open() override;

  void close() override;

  SolenoidStatus getStatus() const override;

  SolenoidConfig getConfig() const override;

private:
  void callback(pcan_message_t* msg);

  PCan& pcan_;
  const pcan_endpoint_id_t address_;

  mutable Mutex mutex_;

  SolenoidStatus status_{};
  SolenoidConfig config_{};
};

}
