/*#pragma once

#include <cstddef>
#include <cstdint>

#include "bytes_le.h"
#include "can_load_cell.hpp"
#include "load_cell.hpp"

namespace proto::can::loadcell {

enum class FrameType : uint8_t {
  kStatus = 1,
  kSamples = 2
};

struct StatusFrame {
  sensors::loadcell::Fault fault = sensors::loadcell::Fault::kNone;
  uint8_t flags = 0;
};

inline constexpr uint8_t kStatusLen = 4;
inline constexpr uint8_t kSamplesHeaderLen = 10;
inline constexpr uint8_t kSampleSize = 2;
inline constexpr uint8_t kSamplesMaxLen =
    kSamplesHeaderLen + kSampleSize * sensors::loadcell::LoadCellBatchSamples;

inline constexpr uint8_t kStatusFlagOverrun = 1u << 0;
inline constexpr uint8_t kSamplesFlagSourceOverrun = 1u << 0;

inline void encode_status(uint8_t* out, const StatusFrame& st) {
  out[0] = static_cast<uint8_t>(FrameType::kStatus);

  const uint16_t f = static_cast<uint16_t>(st.fault);
  pack::wr_u16(out, 1, f);

  out[3] = st.flags;
}

inline bool decode_status(const uint8_t* data, const std::size_t len, StatusFrame& st_out) {
  if (!data) return false;
  if (len < kStatusLen) return false;
  if (static_cast<FrameType>(data[0]) != FrameType::kStatus) return false;

  st_out.fault = static_cast<sensors::loadcell::Fault>(pack::rd_u16(data, 1));
  st_out.flags = data[3];

  return true;
}

inline std::size_t encode_samples(uint8_t* out, const sensors::loadcell::LoadCellBatch& batch) {
  out[0] = static_cast<uint8_t>(FrameType::kSamples);

  pack::wr_u32(out, 1, batch.t0_ms);
  pack::wr_u16(out, 5, batch.dt_us);

  const std::size_t count =
      (batch.count > sensors::loadcell::LoadCellBatchSamples)
          ? sensors::loadcell::LoadCellBatchSamples
          : static_cast<std::size_t>(batch.count);

  out[7] = static_cast<uint8_t>(count);
  out[8] = batch.seq;

  uint8_t flags = 0;
  if (batch.overrun) {
    flags |= kSamplesFlagSourceOverrun;
  }
  out[9] = flags;

  for (std::size_t i = 0; i < count; ++i) {
    pack::wr_u16(
        out,
        static_cast<int>(kSamplesHeaderLen + kSampleSize * i),
        static_cast<uint16_t>(batch.samples[i]));
  }

  return kSamplesHeaderLen + kSampleSize * count;
}

inline bool decode_samples(const uint8_t* data,
                           const std::size_t len,
                           sensors::loadcell::LoadCellBatch& batch_out) {
  if (!data) return false;
  if (len < kSamplesHeaderLen) return false;
  if (static_cast<FrameType>(data[0]) != FrameType::kSamples) return false;

  batch_out.t0_ms = pack::rd_u32(data, 1);
  batch_out.dt_us = pack::rd_u16(data, 5);
  batch_out.count = data[7];
  batch_out.seq = data[8];
  batch_out.overrun = (data[9] & kSamplesFlagSourceOverrun) != 0u;

  if (batch_out.count > sensors::loadcell::LoadCellBatchSamples) {
    return false;
  }

  const std::size_t expected_len =
      kSamplesHeaderLen + kSampleSize * static_cast<std::size_t>(batch_out.count);
  if (len < expected_len) {
    return false;
  }

  for (std::size_t i = 0; i < batch_out.count; ++i) {
    batch_out.samples[i] = static_cast<int16_t>(
        pack::rd_u16(data, static_cast<int>(kSamplesHeaderLen + kSampleSize * i)));
  }

  for (std::size_t i = batch_out.count; i < sensors::loadcell::LoadCellBatchSamples; ++i) {
    batch_out.samples[i] = 0;
  }

  return true;
}

}*/