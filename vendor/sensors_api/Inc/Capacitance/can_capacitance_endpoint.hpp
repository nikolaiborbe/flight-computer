#pragma once

#include "capacitance_sensor.hpp"
#include "local_capacitance_sensor.hpp"
#include "pcan_core.hpp"

namespace sensors::capacitance {

struct EndpointConfig {
  uint32_t status_period_us = 500000;
};

class CanCapacitanceEndpoint final : public CapacitanceSensor {
public:
  CanCapacitanceEndpoint(
      PCan& pcan,
      pcan_endpoint_id_t addr,
      Fdc1004& fdc,
      Fdc1004Measurement measurement,
      Fdc1004Channel channel,
      const SensorConfig& sensor_cfg = {},
      EndpointConfig endpoint_cfg = {});

  void init();

  float getCapacitance() const override;

  FaultFlags getFaults() const override;

  SensorConfig getConfig() const override;

  void update(uint64_t now_us) override;

  bool popSamples(CapacitanceBatch& out) override;

private:
  void callback(pcan_message_t* msg);

  void sendStatusIfNeeded(uint64_t now_us);

  void sendAvailableSamples();

  PCan& pcan_;
  pcan_endpoint_id_t addr_;

  LocalCapacitanceSensor sensor_;
  EndpointConfig endpoint_cfg_;

  uint64_t last_status_us_ = 0;
};

}