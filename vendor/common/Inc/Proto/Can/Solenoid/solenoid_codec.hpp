#pragma once

#include "solenoid.hpp"
#include "bytes_le.h"

namespace proto::can::solenoid {

enum class FrameType : uint8_t {
  OpenCommand  = 1,
  CloseCommand = 2,
  Status       = 3,
  Config       = 4
};

inline constexpr uint8_t OpenCmdLen  = 1;
inline constexpr uint8_t CloseCmdLen = 1;
inline constexpr uint8_t StatusLen   = 4;
inline constexpr uint8_t ConfigLen   = 10;

inline void encode_cmd_open(uint8_t* out) {
  out[0] = static_cast<uint8_t>(FrameType::OpenCommand);
}

inline bool decode_cmd_open(const uint8_t* data, const size_t len) {
  if (len < OpenCmdLen) return false;
  return static_cast<FrameType>(data[0]) == FrameType::OpenCommand;
}

inline void encode_cmd_close(uint8_t* out) {
  out[0] = static_cast<uint8_t>(FrameType::CloseCommand);
}

inline bool decode_cmd_close(const uint8_t* data, const size_t len) {
  if (len < CloseCmdLen) return false;
  return static_cast<FrameType>(data[0]) == FrameType::CloseCommand;
}

inline void encode_status(uint8_t* out, const actuators::solenoid::SolenoidStatus& status) {
  using namespace actuators::solenoid;

  out[0] = static_cast<uint8_t>(FrameType::Status);

  uint8_t flags = 0;
  flags |= (static_cast<uint8_t>(status.power_state) & 0x3u) << 0;
  flags |= (static_cast<uint8_t>(status.opening)     & 0x3u) << 2;
  out[1] = flags;

  float ma_f = status.current_ma;
  if (ma_f > 32767.0f) ma_f = 32767.0f;
  if (ma_f < -32768.0f) ma_f = -32768.0f;
  const int16_t ma = static_cast<int16_t>(ma_f); // trunc is fine
  pack::wr_i16(out, 2, ma);
}

inline bool decode_status(const uint8_t* data, const size_t len,
                          actuators::solenoid::SolenoidStatus& status_out) {
  using namespace actuators::solenoid;

  if (len < StatusLen) return false;
  if (static_cast<FrameType>(data[0]) != FrameType::Status) return false;

  const uint8_t flags = data[1];

  status_out.power_state = static_cast<PowerState>((flags >> 0) & 0x3u);
  status_out.opening     = static_cast<Opening>((flags >> 2) & 0x3u);
  status_out.current_ma  = static_cast<float>(pack::rd_i16(data, 2));

  return true;
}

inline void encode_config(uint8_t* out, const actuators::solenoid::SolenoidConfig& cfg) {
  out[0] = static_cast<uint8_t>(FrameType::Config);
  out[1] = static_cast<uint8_t>(cfg.default_state);

  pack::wr_f32(out, 2, cfg.current_sense.rsense_ohm);
  pack::wr_f32(out, 6, cfg.current_sense.sense_ratio_k);
}

inline bool decode_config(const uint8_t* data, const size_t len,
                          actuators::solenoid::SolenoidConfig& cfg_out) {
  using namespace actuators::solenoid;

  if (len < ConfigLen) return false;
  if (static_cast<FrameType>(data[0]) != FrameType::Config) return false;

  cfg_out.default_state = static_cast<DefaultState>(data[1]);

  cfg_out.current_sense.rsense_ohm   = pack::rd_f32(data, 2);
  cfg_out.current_sense.sense_ratio_k = pack::rd_f32(data, 6);

  return true;
}

}
