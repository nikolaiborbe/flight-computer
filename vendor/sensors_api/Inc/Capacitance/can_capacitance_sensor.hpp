#pragma once

#include "capacitance_sensor.hpp"
#include "pcan_core.hpp"
#include "ring_buffer.hpp"
#include "mutex.hpp"

namespace sensors::capacitance {

inline constexpr uint32_t CommsTimeoutUs = 2000000;

enum class FrameType : uint8_t {
  Status = 0,
  Samples = 1,
};

/**
 * #packed
 */
struct StatusFrame {
  FrameType frame_type = FrameType::Status;
  FaultFlags faults = 0;
  SensorConfig sensor_config{};
};

class CanCapacitanceSensor final : public CapacitanceSensor {
public:

  CanCapacitanceSensor(PCan& pcan, pcan_endpoint_id_t address);

  float getCapacitance() const override;

  FaultFlags getFaults() const override;

  SensorConfig getConfig() const override;

  void update(uint64_t now_us) override;

  bool popSamples(CapacitanceBatch& out) override;

private:
  void callback(pcan_message_t* msg);

  void handleStatus(const uint8_t* payload, std::size_t len);

  void handleSamples(const uint8_t* payload, std::size_t len);

  static float cpfToPf(int16_t cpf);

  mutable Mutex mutex_{};

  PCan& pcan_;
  pcan_endpoint_id_t address_;

  float offset_pf_ = 0.0f;
  float raw_measurement_pf_ = 0.0f;
  float adjusted_measurement_pf_ = 0.0f;

  FaultFlags faults_ = toFlags(Fault::CommFault);

  uint64_t last_rx_us_ = 0;
  uint8_t last_batch_seq_ = 0;
  bool batch_seq_valid_ = false;
  uint32_t rx_drops_ = 0;

  SensorConfig sensor_config_{};

  RingBuffer<CapacitanceBatch, 16> batch_buffer_{};
};

}
