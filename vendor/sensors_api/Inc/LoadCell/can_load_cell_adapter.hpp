/*#pragma once

#include <cstddef>
#include <cstdint>

#include "adc_load_cell.hpp"
#include "can_load_cell.hpp"
#include "pcan_core.hpp"

namespace sensors::loadcell {

struct AdapterConfig {
  uint16_t status_period_us = 50000;
  bool stop_samples_on_fault = false;
};

class CanLoadCellAdapter {
public:
  CanLoadCellAdapter(PCan& pcan,
                     pcan_endpoint_id_t addr,
                     AdcLoadCell& sensor,
                     AdapterConfig cfg);

  void update(uint64_t now_us);

private:
  void collectSamples();
  void sendStatusIfNeeded(uint64_t now_us);
  void sendOneSamplesFrame();

  void publishStatus(Fault fault) const;
  void publishSamples(const LoadCellBatch& batch) const;

  PCan& pcan_;
  pcan_endpoint_id_t addr_;
  AdcLoadCell& sensor_;
  AdapterConfig cfg_;

  AdcLoadCell::Reader reader_{};

  LoadCellSample frame_samples_[LoadCellBatchSamples]{};
  std::size_t frame_count_ = 0;
  bool frame_overrun_ = false;

  Fault last_fault_ = Fault::kInvalidInput;
  uint64_t last_status_us_ = 0;
  uint8_t next_seq_ = 0;
};

}*/