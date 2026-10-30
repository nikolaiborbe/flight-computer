#pragma once

#include <cstddef>

#include "bytes_le.h"
#include "can_capacitance_sensor.hpp"
#include "capacitance_sensor.hpp"

namespace proto::can::capacitance {

inline constexpr uint8_t StatusHeaderLen = 3;  // FrameType(1) + faults(2)
inline constexpr uint8_t SamplesHeaderLen = 9;

// sensors::capacitance::SensorConfig currently carries no fields, so the status
// frame is just the header (fault flags). Kept for parity with the pressure
// codec and easy future extension.
inline constexpr uint8_t ConfigLen = 0;

inline constexpr uint8_t StatusLen = StatusHeaderLen + ConfigLen;

inline constexpr uint8_t OneSampleLen = 6;

inline constexpr uint8_t SamplesMaxLen =
    SamplesHeaderLen + OneSampleLen * sensors::capacitance::BatchSampleCount;

inline constexpr uint8_t SetConfigLen = 1 + ConfigLen;

inline void encode_config(uint8_t* /*out*/,
                          const sensors::capacitance::SensorConfig& /*cfg*/) {
  // No fields to encode yet; placeholder for parity with the pressure codec.
}

inline std::size_t encode_status(uint8_t* out,
                                 sensors::capacitance::FaultFlags faults,
                                 const sensors::capacitance::SensorConfig& cfg) {
  out[0] = static_cast<uint8_t>(sensors::capacitance::FrameType::Status);
  pack::wr_u16(out, 1, faults);
  encode_config(out + StatusHeaderLen, cfg);
  return StatusLen;
}

inline bool decode_status(const uint8_t* data,
                          std::size_t len,
                          sensors::capacitance::StatusFrame& status_out) {
  if (!data || len < StatusLen) return false;

  if (static_cast<sensors::capacitance::FrameType>(data[0]) !=
      sensors::capacitance::FrameType::Status) {
    return false;
  }

  status_out.frame_type = sensors::capacitance::FrameType::Status;
  status_out.faults = pack::rd_u16(data, 1);
  // sensor_config has no fields to decode yet.

  return true;
}

inline std::size_t encode_samples(
    uint8_t* out,
    const sensors::capacitance::CapacitanceBatch& batch) {
  const std::size_t count =
      batch.count > sensors::capacitance::BatchSampleCount
          ? sensors::capacitance::BatchSampleCount
          : static_cast<std::size_t>(batch.count);

  out[0] = static_cast<uint8_t>(sensors::capacitance::FrameType::Samples);

  const uint32_t t0_ms = static_cast<uint32_t>(batch.t0_us / 1000u);

  pack::wr_u32(out, 1, t0_ms);
  pack::wr_u16(out, 5, batch.dt_us > 0xFFFFu ? 0xFFFFu : static_cast<uint16_t>(batch.dt_us));

  out[7] = static_cast<uint8_t>(count);
  out[8] = batch.seq;

  for (std::size_t i = 0; i < count; ++i) {
    const int base = static_cast<int>(SamplesHeaderLen + OneSampleLen * i);

    pack::wr_i16(out, base + 0, batch.samples[i].offset_cpf);
    pack::wr_i16(out, base + 2, batch.samples[i].raw_cpf);
    pack::wr_i16(out, base + 4, batch.samples[i].adjusted_cpf);
  }

  return SamplesHeaderLen + OneSampleLen * count;
}

inline std::size_t encode_capacitance_packet(
    uint8_t* out,
    uint8_t id,
    const sensors::capacitance::CapacitanceBatch& batch) {
  constexpr std::size_t SensorIdLen = 1;
  constexpr std::size_t BatchOffset = SensorIdLen;
  constexpr std::size_t SamplesOffset = BatchOffset + 8 + 4 + 1 + 1;

  const std::size_t count =
      batch.count > sensors::capacitance::BatchSampleCount
          ? sensors::capacitance::BatchSampleCount
          : static_cast<std::size_t>(batch.count);

  out[0] = id;

  pack::wr_u64(out, BatchOffset + 0, batch.t0_us);
  pack::wr_u32(out, BatchOffset + 8, batch.dt_us);

  out[BatchOffset + 12] = static_cast<uint8_t>(count);
  out[BatchOffset + 13] = batch.seq;

  for (std::size_t i = 0; i < count; ++i) {
    const int base = static_cast<int>(SamplesOffset + OneSampleLen * i);

    pack::wr_i16(out, base + 0, batch.samples[i].offset_cpf);
    pack::wr_i16(out, base + 2, batch.samples[i].raw_cpf);
    pack::wr_i16(out, base + 4, batch.samples[i].adjusted_cpf);
  }

  return SamplesOffset + OneSampleLen * count;
}

inline bool decode_samples(
    const uint8_t* data,
    std::size_t len,
    sensors::capacitance::CapacitanceBatch& batch_out) {
  if (!data || len < SamplesHeaderLen) return false;

  if (static_cast<sensors::capacitance::FrameType>(data[0]) !=
      sensors::capacitance::FrameType::Samples) {
    return false;
  }

  const uint32_t t0_ms = pack::rd_u32(data, 1);

  batch_out.t0_us = static_cast<uint64_t>(t0_ms) * 1000u;
  batch_out.dt_us = pack::rd_u16(data, 5);
  batch_out.count = data[7];
  batch_out.seq = data[8];

  if (batch_out.count > sensors::capacitance::BatchSampleCount) {
    return false;
  }

  const std::size_t expected_len =
      SamplesHeaderLen + OneSampleLen * static_cast<std::size_t>(batch_out.count);

  if (len < expected_len) {
    return false;
  }

  for (std::size_t i = 0; i < batch_out.count; ++i) {
    const int base = static_cast<int>(SamplesHeaderLen + OneSampleLen * i);

    batch_out.samples[i].offset_cpf = pack::rd_i16(data, base + 0);
    batch_out.samples[i].raw_cpf = pack::rd_i16(data, base + 2);
    batch_out.samples[i].adjusted_cpf = pack::rd_i16(data, base + 4);
  }

  for (std::size_t i = batch_out.count; i < sensors::capacitance::BatchSampleCount; ++i) {
    batch_out.samples[i] = {};
  }

  return true;
}

}