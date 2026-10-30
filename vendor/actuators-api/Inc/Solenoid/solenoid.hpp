/**
 * @file solenoid.hpp
 * @author Vetle Dahl (vetle.dahl@propulsentnu.no)
 * @author Henrik Berg (henrik.berg@propulsentnu.no)
 *
 * @brief Abstract interface for solenoids.
 *
 * Defines the solenoid API.
 *
 * @copyright Propulse NTNU (c) 2026
 */

#pragma once

#include <cstdint>

namespace actuators::solenoid {

#pragma pack(push, 1)

/**
 * @brief Power state of the solenoid.
 *
 * This reflects the actual electrical state of the solenoid as managed by the
 * implementation.
 */
enum class PowerState : uint8_t { Unknown = 0, Off = 1, On = 2 };

/**
 * @brief Default mechanical state when the solenoid is not energized.
 */
enum class DefaultState : uint8_t { Closed = 1, Open = 2 };

/**
 * @brief Current mechanical opening state of the solenoid.
 */
enum class Opening : uint8_t { Unknown = 0, Closed = 1, Open = 2 };

/**
 * @brief Configuration for current sensing.
 *
 * Describes how the analog feedback signal is mapped to a sensed current.
 */
struct CurrentSenseConfig {
  float rsense_ohm = 1000.0f;
  float sense_ratio_k = 2000.0f;
};

/**
 * @brief Configuration for a solenoid.
 *
 * Represents the current configuration used by the implementation.
 */
struct SolenoidConfig {
  DefaultState default_state = DefaultState::Closed;
  CurrentSenseConfig current_sense{};
};

/**
 * @brief Status of a solenoid.
 *
 * Reports the current state of the solenoid, including the electrical power
 * state, the mechanical opening state, and if available, the measured current.
 */
struct SolenoidStatus {
  PowerState power_state = PowerState::Unknown;
  Opening opening = Opening::Unknown;
  float current_ma = 0.0f;
};

#pragma pack(pop)

/**
 * @brief Abstract solenoid interface.
 */
class Solenoid {

public:

  virtual ~Solenoid() = default;

  /**
   * @brief Command the solenoid to open.
   */
  virtual void open() = 0;

  /**
   * @brief Command the solenoid to close.
   */
  virtual void close() = 0;

  /**
   * @brief Returns a snapshot of solenoid status.
   *
   * For proxy implementations this may be the last-known status and could be stale
   * if no recent telemetry has been received.
   *
   * @return SolenoidStatus snapshot.
   */
  virtual SolenoidStatus getStatus() const = 0;

  /**
   * @brief Returns a snapshot of the solenoid configuration.
   *
   * @return SolenoidConfig snapshot.
   */
  virtual SolenoidConfig getConfig() const = 0;

};

}
