#pragma once

#include <cstddef>

#include "bytes_le.h"
#include "pressure_sensor.hpp"
#include "can_pressure_sensor.hpp"

namespace proto::can::pressure {

inline constexpr uint8_t StatusHeaderLen = 3;
inline constexpr uint8_t SamplesHeaderLen = 9;

inline constexpr uint8_t ConfigLen = 4 * 10 + 1;

inline constexpr uint8_t StatusLen = StatusHeaderLen + ConfigLen;

inline constexpr uint8_t SamplesMaxLen =
    SamplesHeaderLen + 2u * sensors::pressure::BatchSampleCount;

inline constexpr uint8_t SetConfigLen = 1 + ConfigLen;

inline void encode_config(uint8_t* out, const sensors::pressure::SensorConfig& cfg) {
  pack::wr_f32(out, 0, cfg.p_min_bar);
  pack::wr_f32(out, 4, cfg.p_max_bar);
  pack::wr_f32(out, 8, cfg.v_min);
  pack::wr_f32(out, 12, cfg.v_max);
  pack::wr_f32(out, 16, cfg.i_min_ma);
  pack::wr_f32(out, 20, cfg.i_max_ma);
  pack::wr_f32(out, 24, cfg.shunt_ohms);
  pack::wr_f32(out, 28, cfg.disconnect_threshold);
  pack::wr_f32(out, 32, cfg.margin);
  pack::wr_u32(out, 36, cfg.sample_period_us);
  out[40] = static_cast<uint8_t>(cfg.output_type);
}

inline bool decode_config(const uint8_t* data,
                          std::size_t len,
                          sensors::pressure::SensorConfig& cfg) {
  if (!data || len < ConfigLen) return false;

  cfg.p_min_bar = pack::rd_f32(data, 0);
  cfg.p_max_bar = pack::rd_f32(data, 4);
  cfg.v_min = pack::rd_f32(data, 8);
  cfg.v_max = pack::rd_f32(data, 12);
  cfg.i_min_ma = pack::rd_f32(data, 16);
  cfg.i_max_ma = pack::rd_f32(data, 20);
  cfg.shunt_ohms = pack::rd_f32(data, 24);
  cfg.disconnect_threshold = pack::rd_f32(data, 28);
  cfg.margin = pack::rd_f32(data, 32);
  cfg.sample_period_us = pack::rd_u32(data, 36);
  cfg.output_type = static_cast<sensors::pressure::OutputType>(data[40]);

  return true;
}

inline std::size_t encode_status(uint8_t* out,
                                 sensors::pressure::FaultFlags faults,
                                 const sensors::pressure::SensorConfig& cfg) {
  out[0] = static_cast<uint8_t>(sensors::pressure::FrameType::Status);
  pack::wr_u16(out, 1, faults);
  encode_config(out + StatusHeaderLen, cfg);
  return StatusLen;
}

inline bool decode_status(const uint8_t* data,
                          std::size_t len,
                          sensors::pressure::StatusFrame& status_out) {
  if (!data || len < StatusLen) return false;
  if (static_cast<sensors::pressure::FrameType>(data[0]) !=
      sensors::pressure::FrameType::Status) {
    return false;
  }

  status_out.frame_type = sensors::pressure::FrameType::Status;
  status_out.faults = pack::rd_u16(data, 1);

  return decode_config(data + StatusHeaderLen,
                       len - StatusHeaderLen,
                       status_out.sensor_config);
}

inline std::size_t encode_samples(uint8_t* out, const sensors::pressure::PressureBatch& batch) {
  const std::size_t count = batch.count > sensors::pressure::BatchSampleCount
          ? sensors::pressure::BatchSampleCount : static_cast<std::size_t>(batch.count);

  out[0] = static_cast<uint8_t>(sensors::pressure::FrameType::Samples);

  const uint32_t t0_ms = static_cast<uint32_t>(batch.t0_us / 1000u);

  pack::wr_u32(out, 1, t0_ms);
  pack::wr_u16(out, 5, batch.dt_us > 0xFFFFu ? 0xFFFFu : static_cast<uint16_t>(batch.dt_us));

  out[7] = static_cast<uint8_t>(count);
  out[8] = batch.seq;

  for (std::size_t i = 0; i < count; ++i) {
    pack::wr_i16(out, static_cast<int>(SamplesHeaderLen + 2u * i), batch.samples[i]);
  }

  return SamplesHeaderLen + 2u * count;
}

inline std::size_t encode_pressure_packet(
      uint8_t* out,
      uint8_t id,
      const sensors::pressure::PressureBatch& batch
  ) {
  constexpr std::size_t SensorIdLen = 1;
  constexpr std::size_t BatchOffset = SensorIdLen;
  constexpr std::size_t SamplesOffset = BatchOffset + 8 + 4 + 1 + 1;

  const std::size_t count = batch.count > sensors::pressure::BatchSampleCount
          ? sensors::pressure::BatchSampleCount : static_cast<std::size_t>(batch.count);

  out[0] = id;

  pack::wr_u64(out, BatchOffset + 0, batch.t0_us);
  pack::wr_u32(out, BatchOffset + 8, batch.dt_us);

  out[BatchOffset + 12] = static_cast<uint8_t>(count);
  out[BatchOffset + 13] = batch.seq;

  for (std::size_t i = 0; i < count; ++i) {
    pack::wr_i16(out, static_cast<int>(SamplesOffset + 2u * i), batch.samples[i]);
  }

  return SamplesOffset + 2u * count;
}

inline bool decode_samples(const uint8_t* data,
                             std::size_t len,
                             sensors::pressure::PressureBatch& batch_out) {
  if (!data || len < SamplesHeaderLen) {
    return false;
  }

  if (static_cast<sensors::pressure::FrameType>(data[0]) !=
      sensors::pressure::FrameType::Samples) {
    return false;
      }

  const uint32_t t0_ms = pack::rd_u32(data, 1);

  batch_out.t0_us = static_cast<uint64_t>(t0_ms) * 1000u;
  batch_out.dt_us = pack::rd_u16(data, 5);
  batch_out.count = data[7];
  batch_out.seq = data[8];

  if (batch_out.count > sensors::pressure::BatchSampleCount) {
    return false;
  }

  const std::size_t expected_len = SamplesHeaderLen + 2u * static_cast<std::size_t>(batch_out.count);

  if (len < expected_len) {
    return false;
  }

  for (std::size_t i = 0; i < batch_out.count; ++i) {
    batch_out.samples[i] = pack::rd_i16(data, static_cast<int>(SamplesHeaderLen + 2u * i));
  }

  for (std::size_t i = batch_out.count; i < sensors::pressure::BatchSampleCount; ++i) {
    batch_out.samples[i] = 0;
  }

  return true;
}

inline std::size_t encode_set_config(uint8_t* out,
                                       const sensors::pressure::SensorConfig& cfg) {
  out[0] = static_cast<uint8_t>(sensors::pressure::FrameType::SetConfig);
  encode_config(out + 1, cfg);
  return SetConfigLen;
}

inline bool decode_set_config(const uint8_t* data,
                                std::size_t len,
                                sensors::pressure::SensorConfig& cfg_out) {
  if (!data || len < SetConfigLen) return false;

  if (static_cast<sensors::pressure::FrameType>(data[0]) !=
      sensors::pressure::FrameType::SetConfig) {
    return false;
      }

  return decode_config(data + 1, len - 1, cfg_out);
}

}
