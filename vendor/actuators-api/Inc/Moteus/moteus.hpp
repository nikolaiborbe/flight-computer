// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#if defined(STM32G473xx) || defined(STM32G431xx)
#include "stm32g4xx_hal_fdcan.h"
#elif defined(STM32H753xx) || defined(STM32H723xx)
#include "stm32h7xx_hal_fdcan.h"
#else
// Throw and error if we don't recognize the MCU to be safe. If its
// in the same family as one of the above, it can most likely just
// be added to the check above. However, it must be double checked
// and tested that the FDCAN peripherals work identically. In any
// case, open an issue or PR on GitLab if suppport is needed.
#error "Moteus driver is not supported on this MCU."
#endif

#include "moteus_protocol.hpp"
#include "pcan.hpp"

#include <cmath>
#include <cstring>

namespace mm = mjbots::moteus;

/// This is the primary interface to a moteus controller.  One
/// instance of this class should be created per controller that is
/// commanded or monitored.
///
/// The primary control functions each have 3 possible forms:
///
///  1. A "Make" variant which constructs a CanFdFrame to be used in a
///     later call to Transport::Cycle.
///
///  2. A "Set" variant which sends a command to the controller and
///     waits for a response in a blocking manner. edit: This was removed
///     because wtf, who wants to block the whole thread for a single response?
///
///  3. A "Begin" variant which sends a command and requires that the
///     user call Poll() regularly to check for a response, and then
///     retrieve that response from Moteus::last_result().
class Moteus {
public:
  using CanFdFrame = mm::CanFdFrame;
  static constexpr int kDiagnosticTimeoutUs = 4000;

  using Query = mm::Query;
  using PositionMode = mm::PositionMode;
  using VFOCMode = mm::VFOCMode;
  using CurrentMode = mm::CurrentMode;
  using StayWithinMode = mm::StayWithinMode;

  static constexpr mm::Resolution kInt8 = mm::Resolution::kInt8;
  static constexpr mm::Resolution kInt16 = mm::Resolution::kInt16;
  static constexpr mm::Resolution kInt32 = mm::Resolution::kInt32;
  static constexpr mm::Resolution kFloat = mm::Resolution::kFloat;

  struct Options {
    // The ID of the servo to communicate with.
    int8_t id;

    // The source ID to use for the commanding node (i.e. the host or
    // master).
    //int8_t source = 256; This generates a warning and converts into a 0 anyway
    int8_t source = 0;

    mm::Query::Format query_format;
    

    // Use the given prefix for all CAN IDs.
    uint16_t can_prefix = 0x0000;

    // Disable BRS on outgoing frames.
    bool disable_brs = true;

    // Request the configured set of registers as a query with every
    // command.
    bool default_query = true;

    uint16_t min_rcv_wait_us = 2000;

    Options() {}
  };
  PCan &pcan;
  Moteus(PCan& pcan, const Options &options = {})
      : pcan(pcan), options_(options) {
    mm::CanData can_data;
    mm::WriteCanData query_write(&can_data);
    mm::Query::Make(&query_write, options_.query_format);
    

    query_size_ = can_data.size;
    query_data_ = reinterpret_cast<char *>(realloc(query_data_, query_size_));
    ::memcpy(query_data_, &can_data.data[0], query_size_);
  }
  

  struct Result {
    unsigned long timestamp = 0;
    CanFdFrame frame;
    mm::Query::Result values;
  };

  // The most recent result from any command.
  const Result &last_result() const { return last_result_; }

  /////////////////////////////////////////
  // Query

  CanFdFrame MakeQuery(const mm::Query::Format *format_override = nullptr) {
    return MakeFrame(mm::EmptyMode(), {}, {},
                     format_override == nullptr ? &options_.query_format
                                                : format_override);
  }

  /////////////////////////////////////////
  // StopMode

  CanFdFrame MakeStop(const mm::Query::Format *query_override = nullptr) {
    return MakeFrame(mm::StopMode(), {}, {}, query_override);
  }

  void BeginStop(const mm::Query::Format *query_override = nullptr) {
    BeginSingleCommand(MakeStop(query_override));
  }
  
  void BeginQuery(const mm::Query::Format *query_override = nullptr) {
    BeginSingleCommand(MakeQuery(query_override));
  }

  /////////////////////////////////////////
  // BrakeMode

  CanFdFrame MakeBrake(const mm::Query::Format *query_override = nullptr) {
    return MakeFrame(mm::BrakeMode(), {}, {}, query_override);
  }

  void BeginBrake(const mm::Query::Format *query_override = nullptr) {
    BeginSingleCommand(MakeBrake(query_override));
  }

  /////////////////////////////////////////
  // PositionMode

  CanFdFrame
  MakePosition(const mm::PositionMode::Command &cmd,
               const mm::PositionMode::Format *command_override = nullptr,
               const mm::Query::Format *query_override = nullptr) {
    return MakeFrame(mm::PositionMode(), cmd,
                     (command_override == nullptr ? mm::PositionMode::Format()
                                                  : *command_override),
                     query_override);
  }

  void BeginPosition(const mm::PositionMode::Command &cmd,
                     const mm::PositionMode::Format *command_override = nullptr,
                     const mm::Query::Format *query_override = nullptr) {
    BeginSingleCommand(MakePosition(cmd, command_override, query_override));
  }

  /////////////////////////////////////////
  // VFOCMode

  CanFdFrame MakeVFOC(const mm::VFOCMode::Command &cmd,
                      const mm::VFOCMode::Format *command_override = nullptr,
                      const mm::Query::Format *query_override = nullptr) {
    return MakeFrame(mm::VFOCMode(), cmd,
                     (command_override == nullptr ? mm::VFOCMode::Format()
                                                  : *command_override),
                     query_override);
  }

  void BeginVFOC(const mm::VFOCMode::Command &cmd,
                 const mm::VFOCMode::Format *command_override = nullptr,
                 const mm::Query::Format *query_override = nullptr) {
    BeginSingleCommand(MakeVFOC(cmd, command_override, query_override));
  }

  /////////////////////////////////////////
  // CurrentMode

  CanFdFrame
  MakeCurrent(const mm::CurrentMode::Command &cmd,
              const mm::CurrentMode::Format *command_override = nullptr,
              const mm::Query::Format *query_override = nullptr) {
    return MakeFrame(mm::CurrentMode(), cmd,
                     (command_override == nullptr ? mm::CurrentMode::Format()
                                                  : *command_override),
                     query_override);
  }

  void BeginCurrent(const mm::CurrentMode::Command &cmd,
                    const mm::CurrentMode::Format *command_override = nullptr,
                    const mm::Query::Format *query_override = nullptr) {
    BeginSingleCommand(MakeCurrent(cmd, command_override, query_override));
  }

  /////////////////////////////////////////
  // StayWithinMode

  CanFdFrame
  MakeStayWithin(const mm::StayWithinMode::Command &cmd,
                 const mm::StayWithinMode::Format *command_override = nullptr,
                 const mm::Query::Format *query_override = nullptr) {
    return MakeFrame(mm::StayWithinMode(), cmd,
                     (command_override == nullptr ? mm::StayWithinMode::Format()
                                                  : *command_override),
                     query_override);
  }

  void
  BeginStayWithin(const mm::StayWithinMode::Command &cmd,
                  const mm::StayWithinMode::Format *command_override = nullptr,
                  const mm::Query::Format *query_override = nullptr) {
    BeginSingleCommand(MakeStayWithin(cmd, command_override, query_override));
  }

  /////////////////////////////////////////
  // OutputNearest

  CanFdFrame
  MakeOutputNearest(const mm::OutputNearest::Command &cmd,
                    const mm::OutputNearest::Format *command_override = nullptr,
                    const mm::Query::Format *query_override = nullptr) {
    return MakeFrame(mm::OutputNearest(), cmd,
                     (command_override == nullptr ? mm::OutputNearest::Format()
                                                  : *command_override),
                     query_override);
  }

  void BeginOutputNearest(
      const mm::OutputNearest::Command &cmd,
      const mm::OutputNearest::Format *command_override = nullptr,
      const mm::Query::Format *query_override = nullptr) {
    BeginSingleCommand(
        MakeOutputNearest(cmd, command_override, query_override));
  }

  /////////////////////////////////////////
  // OutputExact

  CanFdFrame
  MakeOutputExact(const mm::OutputExact::Command &cmd,
                  const mm::OutputExact::Format *command_override = nullptr,
                  const mm::Query::Format *query_override = nullptr) {
    return MakeFrame(mm::OutputExact(), cmd,
                     (command_override == nullptr ? mm::OutputExact::Format()
                                                  : *command_override),
                     query_override);
  }

  void
  BeginOutputExact(const mm::OutputExact::Command &cmd,
                   const mm::OutputExact::Format *command_override = nullptr,
                   const mm::Query::Format *query_override = nullptr) {
    BeginSingleCommand(MakeOutputExact(cmd, command_override, query_override));
  }

  /////////////////////////////////////////
  // RequireReindex

  CanFdFrame MakeRequireReindex(
      const mm::RequireReindex::Command &cmd,
      const mm::RequireReindex::Format *command_override = nullptr,
      const mm::Query::Format *query_override = nullptr) {
    return MakeFrame(mm::RequireReindex(), cmd,
                     (command_override == nullptr ? mm::RequireReindex::Format()
                                                  : *command_override),
                     query_override);
  }

  void BeginRequireReindex(
      const mm::RequireReindex::Command &cmd,
      const mm::RequireReindex::Format *command_override = nullptr,
      const mm::Query::Format *query_override = nullptr) {
    BeginSingleCommand(
        MakeRequireReindex(cmd, command_override, query_override));
  }

  /////////////////////////////////////////
  // RecapturePositionVelocity

  CanFdFrame MakeRecapturePositionVelocity(
      const mm::RecapturePositionVelocity::Command &cmd,
      const mm::RecapturePositionVelocity::Format *command_override = nullptr,
      const mm::Query::Format *query_override = nullptr) {
    return MakeFrame(mm::RecapturePositionVelocity(), cmd,
                     (command_override == nullptr
                          ? mm::RecapturePositionVelocity::Format()
                          : *command_override),
                     query_override);
  }

  void BeginRecapturePositionVelocity(
      const mm::RecapturePositionVelocity::Command &cmd,
      const mm::RecapturePositionVelocity::Format *command_override = nullptr,
      const mm::Query::Format *query_override = nullptr) {
    BeginSingleCommand(
        MakeRecapturePositionVelocity(cmd, command_override, query_override));
  }

  /////////////////////////////////////////
  // Diagnostic channel operations

  enum DiagnosticReplyMode {
    kExpectOK,
    kExpectSingleLine,
  };

  /////////////////////////////////////////
  // Non-command related methods

  /// Look for a response to a previous command.  Return true if one
  /// has been received.  The parsed results can be seen in
  /// Moteus::last_result()
  bool rx_callback(pcan_message_t *message) {

    // if (rxHeader.Identifier != static_cast<uint32_t>(options_.id)) {
    //   return false;
    // }

    auto &cf = last_result_.frame;
    cf.destination = message->raw_id;
    cf.size = message->size;
    memcpy(&cf.data[0], &message->data[0], cf.size);

    if (message->bitrate_switch == FDCAN_BRS_ON) {
      cf.brs = mm::CanFdFrame::kForceOn;
      cf.fdcan_frame = mm::CanFdFrame::kForceOn;
    } else if (message->bitrate_switch == FDCAN_BRS_OFF) {
      cf.brs = mm::CanFdFrame::kForceOff;
      cf.fdcan_frame = mm::CanFdFrame::kForceOn;
    } else {
      cf.brs = mm::CanFdFrame::kForceOff;
      cf.fdcan_frame = mm::CanFdFrame::kForceOff;
    }

    last_result_.values = mm::Query::Parse(&cf.data[0], cf.size);

    return true;
  }

  bool BeginSingleCommand(const mm::CanFdFrame &frame) {
    // FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[64];

    // Copy and pad data
    memcpy(TxData, frame.data, frame.size);
    PadCan(TxData, frame.size);

    // Send the frame (replaces Arduino can_bus_.tryToSend())
    int status = pcan.publish_raw(
      frame.arbitration_id,
      TxData,
      RoundUpDlc(frame.size)
    );

    // Optional: Check if send was successful
    if (status < 0) {
      // Handle error - maybe return false or retry
      return false;
    }

    return frame.reply_required;
  }

private:
  enum ReplyMode {
    kNoReply,
    kReplyRequired,
  };

  CanFdFrame DefaultFrame(ReplyMode reply_mode = kReplyRequired) {
    CanFdFrame result;
    result.destination = options_.id;
    result.reply_required = (reply_mode == kReplyRequired);

    result.arbitration_id = (result.destination) | (result.source << 8) |
                            (result.reply_required ? 0x8000 : 0x0000) |
                            (static_cast<uint32_t>(options_.can_prefix) << 16);
    result.bus = 0;

    return result;
  }

  template <typename CommandType>
  CanFdFrame MakeFrame(const CommandType &,
                       const typename CommandType::Command &cmd,
                       const typename CommandType::Format &fmt,
                       const mm::Query::Format *query_override = nullptr) {
    auto result = DefaultFrame(query_override != nullptr ? kReplyRequired
                               : options_.default_query  ? kReplyRequired
                                                         : kNoReply);

    mm::WriteCanData write_frame(result.data, &result.size);
    CommandType::Make(&write_frame, cmd, fmt);

    if (query_override) {
      mm::Query::Make(&write_frame, *query_override);
    } else if (options_.default_query) {
      ::memcpy(&result.data[result.size], query_data_, query_size_);
      result.size += query_size_;
    }

    return result;
  }

  static int8_t getDLC(int8_t size) {
    if (size <= 0) {
      return FDCAN_DLC_BYTES_0;
    }
    if (size <= 1) {
      return FDCAN_DLC_BYTES_1;
    }
    if (size <= 2) {
      return FDCAN_DLC_BYTES_2;
    }
    if (size <= 3) {
      return FDCAN_DLC_BYTES_3;
    }
    if (size <= 4) {
      return FDCAN_DLC_BYTES_4;
    }
    if (size <= 5) {
      return FDCAN_DLC_BYTES_5;
    }
    if (size <= 6) {
      return FDCAN_DLC_BYTES_6;
    }
    if (size <= 7) {
      return FDCAN_DLC_BYTES_7;
    }
    if (size <= 8) {
      return FDCAN_DLC_BYTES_8;
    }
    if (size <= 12) {
      return FDCAN_DLC_BYTES_12;
    }
    if (size <= 16) {
      return FDCAN_DLC_BYTES_16;
    }
    if (size <= 20) {
      return FDCAN_DLC_BYTES_20;
    }
    if (size <= 24) {
      return FDCAN_DLC_BYTES_24;
    }
    if (size <= 32) {
      return FDCAN_DLC_BYTES_32;
    }
    if (size <= 48) {
      return FDCAN_DLC_BYTES_48;
    }
    if (size <= 64) {
      return FDCAN_DLC_BYTES_64;
    }
    return 0;
  }

 static int8_t RoundUpDlc(int8_t size) {
    if (size <= 0) {
      return 0;
    }
    if (size <= 1) {
      return 1;
    }
    if (size <= 2) {
      return 2;
    }
    if (size <= 3) {
      return 3;
    }
    if (size <= 4) {
      return 4;
    }
    if (size <= 5) {
      return 5;
    }
    if (size <= 6) {
      return 6;
    }
    if (size <= 7) {
      return 7;
    }
    if (size <= 8) {
      return 8;
    }
    if (size <= 12) {
      return 12;
    }
    if (size <= 16) {
      return 16;
    }
    if (size <= 20) {
      return 20;
    }
    if (size <= 24) {
      return 24;
    }
    if (size <= 32) {
      return 32;
    }
    if (size <= 48) {
      return 48;
    }
    if (size <= 64) {
      return 64;
    }
    return 0;
  }


  // Convert Arduino PadCan to STM32 version
  void PadCan(uint8_t *data, const uint8_t size) {
    const auto new_size = RoundUpDlc(size);
    for (int8_t i = size; i < new_size; i++) {
      data[i] = 0x50;
    }
  };
  const Options options_;

  Result last_result_;
  char *query_data_ = nullptr;
  size_t query_size_ = 0;
};