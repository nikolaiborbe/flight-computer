/**
 * @file servo.hpp
 * @author Henrik Berg (henrik.berg@propulsentnu.no)
 *
 * @brief Abstract interface for position-controlled servos.
 *
 * Defines the servo API.
 *
 * @copyright Propulse NTNU (c) 2026
 */

#pragma once

#include <cstdint>

#include "servo_model.hpp"

namespace actuators::servo {

#pragma pack(push, 1)

/**
 * @brief Power state of the servo.
 *
 * This reflects the actual electrical state of the servo as managed by the
 * implementation, and is independent of the power mode.
 */
enum class PowerState : uint8_t { Unknown = 0, Off = 1, Booting = 2, On = 3 };

/**
 * @brief Power management policy for the servo.
 *
 * This policy controls whether the servo is allowed to automatically power
 * down when idle.
 */
enum class PowerMode : uint8_t { KeepOn = 0, AutoOff = 1 };

/**
 * @brief Interpretation of the position command angle.
 *
 * In absolute mode, it specifies an absolute target angle in degrees.
 * In relative mode, it is applied as an offset relative to the current
 * commanded angle.
 */
enum class PositionMode : uint8_t { Absolute = 0, Relative = 1 };

/**
 * @brief Start policy when a new command is submitted.
 *
 * This policy controls how a new position command interacts with any pending or
 * currently active command. When kPreempt is used, a newly submitted command
 * replaces any existing pending or active command. When kNoInterrupt is selected,
 * the new command is rejected if the servo is already busy executing or holding
 * another command.
 */
enum class StartPolicy : uint8_t { Preempt = 0, NoInterrupt = 1 };

/**
 * @brief Result of a wakeup request.
 *
 * Wakeup requests are used to power the servo and allow it to complete booting
 * before motion commands are executed.
 */
enum class WakeupResult : uint8_t { Ok = 0, RejectedBusy = 1, RejectedInvalid = 2 };

/**
 * @brief Result of submitting a position command.
 */
enum class CommandResult : uint8_t {
  Accepted           = 0,
  RejectedQueueFull  = 1,
  RejectedBusy       = 2,
  RejectedNotReady   = 3,
  RejectedInvalid    = 4,
  PreemptedPrevious  = 5
};

/**
* @brief Parameters for requesting servo power.
*/
struct WakeupRequest {
  uint16_t keep_alive_ms = 0;
};

/**
* @brief Configuration for angle measurement.
*
* Describes how the analog feedback signal is mapped to mechanical
* angles.
*/
struct AngleMeasurementConfig {
  float v_at_min  = 0;
  float v_at_max  = 0;
};

struct CurrentSenseConfig {
  float rsense_ohm = 1000.0f;
  float sense_ratio_k = 2000.0f;
};

/**
* @brief Configuration for a servo.
*
* Represents the current configuration used by the
* implementation.
*/
struct ServoConfig {
  ServoModelId model_id = ServoModelId::Unknown;
  PowerMode power_mode = PowerMode::AutoOff;
  uint16_t idle_timeout_ms = 2000u;
  AngleMeasurementConfig angle_measurement{};
  CurrentSenseConfig current_sense{};
};

/**
 * @brief Status of a servo.
 *
 * Reports the current state of the servo, including the most
 * recently commanded angle, the target angle of the active command, and if
 * available, the measured angle.
 */
struct ServoStatus {
  float commanded_angle_deg = 0.0f;
  float target_angle_deg = 0.0f;
  float measured_angle_deg = 0.0f;
  float current_ma = 0.0f;
  PowerState power_state = PowerState::Unknown;
};

/**
* @brief Position command parameters.
*
* A position command describes a desired target angle and optional timing
* behavior.
*/
struct PositionCommand {
  float angle_deg = 0.0f;
  uint16_t duration_ms = 0;
  uint16_t delay_ms = 0;
  PositionMode position_mode = PositionMode::Absolute;
  StartPolicy start_policy = StartPolicy::Preempt;
};

#pragma pack(pop)

/**
* @brief Abstract servo interface.
*/
class Servo {

public:

  virtual ~Servo() = default;

  /**
   * @brief Begin a position command.
   *
   * Implementations may queue the command for later execution, preempt previous
   * commands or reject based on start policy or readiness.
   *
   * @param command Desired position command.
   * @return CommandResult accepted/rejected (+ preempt info).
   */
  virtual CommandResult beginPosition(const PositionCommand& command) = 0;

  /**
   * @brief Request that the servo be powered and ready for motion.
   *
   * @param request Wakeup request parameters.
   * @return WakeupResult indicating whether the request was accepted.
   */
  virtual WakeupResult requestWakeup(const WakeupRequest& request) = 0;

  /**
   * @brief Returns a snapshot of servo status.
   *
   * For proxy implementations this may be the last-known status and could be stale
   * if no recent telemetry has been received.
   *
   * @return ServoStatus snapshot.
   */
  virtual ServoStatus getStatus() const = 0;

  /**
   * @brief Returns the servo model parameters used by the implementation.
   *
   * @return Reference to the current ServoModel.
   */
  virtual const ServoModel& getModel() const = 0;

  /**
   * @brief Returns a snapshot of the servo configuration.
   *
   * @return ServoConfig snapshot.
   */
  virtual ServoConfig getConfig() const = 0;

};

}
