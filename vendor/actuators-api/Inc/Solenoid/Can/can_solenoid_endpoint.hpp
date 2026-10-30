#pragma once

#include <pcan_core.hpp>
#include <gpio_solenoid.hpp>

namespace actuators::solenoid {

class CanSolenoidEndpoint : public Solenoid {
public:

  CanSolenoidEndpoint(PCan& pcan, pcan_node_id_t addr, DigitalOut& enable_pin,
    const SolenoidConfig& config = {});

  void open() override;

  void close() override;

  SolenoidStatus getStatus() const override;

  SolenoidConfig getConfig() const override;

  void update(uint64_t now_ms);

  void onCurrentSample(uint64_t now_ms, float volts);

private:
  void onCommand(pcan_message_t* msg);

  void publishStatus() const;

  void publishConfig() const;

  PCan& pcan_;
  pcan_node_id_t addr_;
  GpioSolenoid solenoid_;

  uint64_t next_status_ms_ = 0;
  uint64_t next_config_ms_ = 0;
};

}
