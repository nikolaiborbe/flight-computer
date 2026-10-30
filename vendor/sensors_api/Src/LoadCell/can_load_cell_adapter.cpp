/*#include "can_load_cell_adapter.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "loadcell_codec.hpp"
#include "units.hpp"

namespace sensors::loadcell {

CanLoadCellAdapter::CanLoadCellAdapter(PCan& pcan,
                                       pcan_endpoint_id_t addr,
                                       AdcLoadCell& sensor,
                                       AdapterConfig cfg)
    : pcan_(pcan),
      addr_(addr),
      sensor_(sensor),
      cfg_(cfg),
      reader_(sensor.makeReader()),
      last_fault_(sensor.getFault()) {
}

void CanLoadCellAdapter::update(uint64_t now_us) {
  collectSamples();
  sendStatusIfNeeded(now_us);

  if (cfg_.stop_samples_on_fault && (sensor_.getFault() != Fault::kNone)) {
    return;
  }

  if (frame_count_ == LoadCellBatchSamples) {
    sendOneSamplesFrame();
  }
}

void CanLoadCellAdapter::collectSamples() {
  while (frame_count_ < LoadCellBatchSamples) {
    const std::size_t need = LoadCellBatchSamples - frame_count_;
    LoadCellSample tmp[LoadCellBatchSamples]{};

    const auto rr = sensor_.read(reader_, tmp, need);

    if (rr.overrun) {
      frame_overrun_ = true;
    }

    if (rr.count == 0) {
      break;
    }

    for (std::size_t i = 0; i < rr.count; ++i) {
      frame_samples_[frame_count_++] = tmp[i];
    }
  }
}

void CanLoadCellAdapter::sendStatusIfNeeded(uint64_t now_us) {
  const Fault fault = sensor_.getFault();

  const bool period_elapsed = (now_us - last_status_us_) >= cfg_.status_period_us;

  if (!period_elapsed) {
    return;
  }

  publishStatus(fault);
  last_fault_ = fault;
  last_status_us_ = now_us;
}

void CanLoadCellAdapter::sendOneSamplesFrame() {
  LoadCellBatch batch{};
  batch.count = static_cast<uint8_t>(LoadCellBatchSamples);
  batch.seq = next_seq_++;
  batch.overrun = frame_overrun_ ? 1 : 0;
  batch.t0_ms = static_cast<uint32_t>(frame_samples_[0].t_us / 1000u);

  const uint32_t dt_total =
      static_cast<uint32_t>(frame_samples_[LoadCellBatchSamples - 1].t_us -
                            frame_samples_[0].t_us);
  const uint32_t avg_dt = dt_total / static_cast<uint32_t>(LoadCellBatchSamples - 1);
  batch.dt_us = (avg_dt > 0xFFFFu) ? 0xFFFFu : static_cast<uint16_t>(avg_dt);

  for (std::size_t i = 0; i < LoadCellBatchSamples; ++i) {
  	batch.samples[i] = frame_samples_[i].weight_dkg;
  }

  publishSamples(batch);

  frame_count_ = 0;
  frame_overrun_ = false;
}

void CanLoadCellAdapter::publishStatus(Fault fault) const {
  proto::can::loadcell::StatusFrame st{};
  st.fault = fault;
  st.flags = 0;

  std::array<uint8_t, proto::can::loadcell::kStatusLen> data{};
  proto::can::loadcell::encode_status(data.data(), st);

  pcan_.publish_telemetry(PCAN_PRIORITY_LOW, addr_.channel_id, data.data(), data.size());
}

void CanLoadCellAdapter::publishSamples(const LoadCellBatch& batch) const {
  std::array<uint8_t, proto::can::loadcell::kSamplesMaxLen> data{};
  const std::size_t len = proto::can::loadcell::encode_samples(data.data(), batch);

  pcan_.publish_telemetry(PCAN_PRIORITY_LOW, addr_.channel_id, data.data(), len);
}

}
*/