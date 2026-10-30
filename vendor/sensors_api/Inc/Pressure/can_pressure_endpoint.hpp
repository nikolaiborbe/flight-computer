#pragma once

#include <cstddef>
#include <cstdint>

#include "adc_pressure_sensor.hpp"
#include "pcan_core.hpp"
#include "pressure_sensor.hpp"

namespace sensors::pressure {

struct EndpointConfig {
	uint32_t status_period_us = 500000;
};

class CanPressureEndpoint final : public PressureSensor {
public:
	CanPressureEndpoint(PCan& pcan,
										 pcan_endpoint_id_t addr,
										 const SensorConfig& sensor_cfg,
										 EndpointConfig adapter_cfg = {});

  void onSample(uint64_t now_us, float volts);

	void update(uint64_t now_us) override;

  float getPressure() const override;

  FaultFlags getFaults() const override;

  void setSamplePeriod(uint32_t sample_period_us) override;

  SensorConfig getConfig() const override;

  bool popSamples(PressureBatch& out) override;

private:
  void callback(pcan_message_t* msg);

  void handleSetConfig(const uint8_t *payload, std::size_t len);

	void sendStatusIfNeeded(uint64_t now_us);

	void sendAvailableSamples();

	PCan& pcan_;
	pcan_endpoint_id_t addr_;

	AdcPressureSensor sensor_;
	EndpointConfig adapter_cfg_;

	uint64_t last_status_us_ = 0;
};

}
