#pragma once

#include <pcan_core.hpp>
#include <pressure_sensor.hpp>

#include "mutex.hpp"
#include "ring_buffer.hpp"

namespace sensors::pressure {

inline constexpr uint32_t CommsTimeoutUs = 2000000; // 2 seconds

enum class FrameType : uint8_t {
  Status = 0,
  Samples = 1,
  SetConfig = 2
};

/**
 * #packed
 */
struct StatusFrame {
  FrameType frame_type = FrameType::Status;
  FaultFlags faults = 0;
  SensorConfig sensor_config{};
};

class CanPressureSensor final : public PressureSensor {
public:
  explicit CanPressureSensor(PCan& pcan, pcan_endpoint_id_t address);

  float getPressure() const override;

  FaultFlags getFaults() const override;

  void setSamplePeriod(uint32_t sample_period_us) override;

  SensorConfig getConfig() const override;

  void update(uint64_t now_us) override;

  bool popSamples(PressureBatch& out) override;

  bool popAbortSamples(PressureBatch& out);

private:
  void callback(pcan_message_t* msg);

  void handleStatus(const uint8_t *payload, std::size_t len);

  void handleSamples(const uint8_t *payload, std::size_t len);

  mutable Mutex mutex_{};

  PCan& pcan_;
  pcan_endpoint_id_t address_;

  float pressure_bar_{0.0f};
  FaultFlags faults_{toFlags(Fault::CommFault)};
  uint64_t last_rx_us_{0};
  uint8_t last_batch_seq_{0};
  bool batch_seq_valid_{false};
  uint32_t rx_drops_{0};

  SensorConfig sensor_config_{};

	RingBuffer<PressureBatch, 16> batch_buffer_{};
  RingBuffer<PressureBatch, 16> abort_buffer_{};
};

}
