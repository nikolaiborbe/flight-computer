/*#pragma once

#include "pcan_core.hpp"
#include "load_cell.hpp"
#include "seqlock.h"
#include "stream_buffer.hpp"

namespace sensors::loadcell {

inline constexpr std::size_t LoadCellBatchSamples = 20;

#pragma pack(push, 1)
struct LoadCellBatch {
  uint32_t t0_ms = 0;
  uint16_t dt_us = 1000;
  uint8_t count = 0;
  uint8_t seq = 0;
  bool overrun = false;
  int16_t samples[LoadCellBatchSamples]{};
};
#pragma pack(pop)

class CanLoadCell final : public LoadCell {
public:
  using Reader = StreamBuffer<LoadCellBatch, 16>::Reader;
  using ReadResult = StreamBuffer<LoadCellBatch, 16>::ReadResult;

  explicit CanLoadCell(PCan& pcan, pcan_endpoint_id_t address);

  float getWeight() const override;

  Fault getFault() const override;

  const Config& getConfig() const override;

  bool isStale(uint64_t now_us) const;

  Reader makeReader() const { return batch_buffer_.makeReader(); }

  ReadResult read(Reader& reader, LoadCellBatch* out, std::size_t max) const {
    return batch_buffer_.read(reader, out, max);
  }

private:
  struct State {
    float load_kg = 0.0f;
    Fault fault = Fault::kCommFault;
    uint64_t last_rx_us = 0;
    uint8_t last_batch_seq = 0;
    bool batch_seq_valid = false;
    uint32_t rx_drops = 0;
  };

  void onTelemetry(pcan_message_t* msg);
  bool handleStatus(uint64_t now_us, const uint8_t* payload, std::size_t len);
  bool handleSamples(uint64_t now_us, const uint8_t* payload, std::size_t len);

  static constexpr uint32_t kStaleTimeoutUs = 100000;

  PCan& pcan_;
  pcan_endpoint_id_t address_;
  Config config_{};
  SeqLock<State> state_snapshot_{};
  StreamBuffer<LoadCellBatch, 16> batch_buffer_{};
};

}*/