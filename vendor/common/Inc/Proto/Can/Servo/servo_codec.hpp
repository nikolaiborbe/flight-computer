#pragma once

#include "can_servo.hpp"
#include "bytes_le.h"
#include "units.hpp"

namespace proto::can::servo {

enum class FrameType : uint8_t {
  PositionCommand = 1,
  WakeupCommand = 2,
  Status = 3,
  Config = 4
};

inline constexpr uint8_t PosCmdLen = 8;
inline constexpr uint8_t WakeCmdLen = 3;
inline constexpr uint8_t StatusLen = 10;
inline constexpr uint8_t ConfigLen = 21;

inline void encode_cmd_position(uint8_t* out, const actuators::servo::PositionCommand& command) {
  using namespace actuators::servo;

  out[0] = static_cast<uint8_t>(FrameType::PositionCommand);

  uint8_t flags = 0;
  if (command.position_mode == PositionMode::Relative) flags |= 1u << 0;
  if (command.start_policy == StartPolicy::NoInterrupt) flags |= 1u << 1;
  out[1] = flags;

  pack::wr_i16(out, 2, units::deg_to_cdeg(command.angle_deg));
  pack::wr_u16(out, 4, command.duration_ms);
  pack::wr_u16(out, 6, command.delay_ms);
}

inline bool decode_cmd_position(const uint8_t* data, const size_t len,
                                actuators::servo::PositionCommand& command_out) {
  using namespace actuators::servo;
  if (len < PosCmdLen) return false;
  if (static_cast<FrameType>(data[0]) != FrameType::PositionCommand) return false;

  const uint8_t flags = data[1];

  command_out.angle_deg = units::cdeg_to_deg(pack::rd_i16(data, 2));
  command_out.duration_ms = pack::rd_u16(data, 4);
  command_out.delay_ms = pack::rd_u16(data, 6);

  command_out.position_mode = (flags & (1u << 0)) ? PositionMode::Relative : PositionMode::Absolute;
  command_out.start_policy = (flags & (1u << 1)) ? StartPolicy::NoInterrupt : StartPolicy::Preempt;

  return true;
}

inline void encode_cmd_wakeup(uint8_t* out,
                                const actuators::servo::WakeupRequest& req) {
  out[0] = static_cast<uint8_t>(FrameType::WakeupCommand);
  pack::wr_u16(out, 1, req.keep_alive_ms);
}

inline bool decode_cmd_wakeup(const uint8_t* data, const size_t len,
                                actuators::servo::WakeupRequest& req_out) {
  if (len < WakeCmdLen) return false;
  if (static_cast<FrameType>(data[0]) != FrameType::WakeupCommand) return false;
  req_out.keep_alive_ms = pack::rd_u16(data, 1);
  return true;
}

inline void encode_status(uint8_t* out, const actuators::servo::ServoStatus& status) {
  using namespace actuators::servo;

  out[0] = static_cast<uint8_t>(FrameType::Status);

  uint8_t flags = 0;
  flags |= static_cast<uint8_t>(status.power_state) & 0x3u;
  out[1] = flags;

  pack::wr_i16(out, 2, units::deg_to_cdeg(status.commanded_angle_deg));
  pack::wr_i16(out, 4, units::deg_to_cdeg(status.target_angle_deg));
  pack::wr_i16(out, 6, units::deg_to_cdeg(status.measured_angle_deg));

  float ma_f = status.current_ma;
  if (ma_f > 32767.0f) ma_f = 32767.0f;
  if (ma_f < -32768.0f) ma_f = -32768.0f;
  const int16_t ma = static_cast<int16_t>(ma_f); // trunc is fine
  pack::wr_i16(out, 8, ma);
}

inline bool decode_status(const uint8_t* data, const size_t len, actuators::servo::ServoStatus& status_out) {
  using namespace actuators::servo;

  if (len < StatusLen) return false;
  if (static_cast<FrameType>(data[0]) != FrameType::Status) return false;

  const uint8_t flags = data[1];

  status_out.power_state = static_cast<PowerState>(flags & 0x3u);
  status_out.commanded_angle_deg = units::cdeg_to_deg(pack::rd_i16(data, 2));
  status_out.target_angle_deg = units::cdeg_to_deg(pack::rd_i16(data, 4));
  status_out.measured_angle_deg = units::cdeg_to_deg(pack::rd_i16(data, 6));
  status_out.current_ma = static_cast<float>(pack::rd_i16(data, 8));

  return true;
}

inline void encode_config(uint8_t* out, const actuators::servo::ServoConfig& cfg) {
  using namespace actuators::servo;

  out[0] = static_cast<uint8_t>(FrameType::Config);
  out[1] = static_cast<uint8_t>(cfg.model_id);
  out[2] = static_cast<uint8_t>(cfg.power_mode);

  pack::wr_u16(out, 3, cfg.idle_timeout_ms);

  pack::wr_f32(out, 5,  cfg.angle_measurement.v_at_min);
  pack::wr_f32(out, 9,  cfg.angle_measurement.v_at_max);

  pack::wr_f32(out, 13, cfg.current_sense.rsense_ohm);
  pack::wr_f32(out, 17, cfg.current_sense.sense_ratio_k);
}

inline bool decode_config(const uint8_t* data, const size_t len,
                          actuators::servo::ServoConfig& cfg_out) {
  using namespace actuators::servo;

  if (len < ConfigLen) return false;
  if (static_cast<FrameType>(data[0]) != FrameType::Config) return false;

  cfg_out.model_id = static_cast<ServoModelId>(data[1]);
  cfg_out.power_mode = static_cast<PowerMode>(data[2]);
  cfg_out.idle_timeout_ms = pack::rd_u16(data, 3);

  cfg_out.angle_measurement.v_at_min = pack::rd_f32(data, 5);
  cfg_out.angle_measurement.v_at_max = pack::rd_f32(data, 9);

  cfg_out.current_sense.rsense_ohm = pack::rd_f32(data, 13);
  cfg_out.current_sense.sense_ratio_k = pack::rd_f32(data, 17);

  return true;
}

}
