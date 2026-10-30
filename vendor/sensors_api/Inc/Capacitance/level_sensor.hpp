#pragma once

#include <pcan_core.hpp>

#include "capacitance_sensor.hpp"
#include "mutex.hpp"
#include "seqlock.h"

namespace sensors::capacitance {

inline constexpr std::size_t LevelWindowSampleCount = 10;
inline constexpr uint32_t LevelSampleSpacingMs = 10;

#pragma pack(push, 1)
struct LevelBatch {
  uint64_t t0_us;
  uint32_t dt_us;
  uint8_t count;
  uint8_t seq;
  float heights_m[LevelWindowSampleCount];
};
#pragma pack(pop)

/**
 * @brief Derives a liquid level (height) from three capacitance sensors.
 *
 * Subscribes to the main, gas-reference and liquid-reference endpoints and caches
 * the latest capacitance from each. update() runs as a fast loop, computing a height
 * and writing it into the current slot of a fixed window; each completed window is
 * published whole. readLatest() returns the most recent complete window.
 */
class LevelSensor {
public:
  LevelSensor(PCan& pcan,
              pcan_endpoint_id_t main,
              pcan_endpoint_id_t gas,
              pcan_endpoint_id_t liquid);

  void update(uint64_t now_us);

  bool readLatest(LevelBatch& out) const;

private:
  void onMain(pcan_message_t* msg);
  void onGas(pcan_message_t* msg);
  void onLiquid(pcan_message_t* msg);

  static bool latestPf(pcan_message_t* msg, float& out_pf);

  PCan& pcan_;

  mutable Mutex mutex_{};
  float c_main_ = 0.0f;
  float c_gas_ = 0.0f;
  float c_liquid_ = 0.0f;
  bool have_main_ = false;
  bool have_gas_ = false;
  bool have_liquid_ = false;

  LevelBatch working_{};
  uint32_t working_window_ = 0;
  bool working_valid_ = false;
  uint8_t window_seq_ = 0;

  SeqLock<LevelBatch> snapshot_{};
};

}
