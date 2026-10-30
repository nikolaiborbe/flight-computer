/**
 * @file timebase.hpp
 *
 * @brief Monotonic timebase (C++ convenience wrapper).
 *
 * This header provides a C++ namespace wrapper around the C ABI
 * defined in @ref timebase.h.
 *
 * Usage:
 *   - C code includes @ref timebase.h and calls timebase_init(), etc.
 *   - C++ code may include this header and call timebase::millis(), etc.
 *
 * @copyright Propulse NTNU (c) 2026
 */

#pragma once
#include "timebase.h"

namespace timebase {

/**
 * @brief Initialize the timebase with a HAL timer handle.
 *
 * @param htim Pointer to a HAL TIM_HandleTypeDef (e.g. &htim5).
 */
inline void init(void* htim) {
  timebase_init(htim);
}

/** @brief Returns monotonic time in microseconds since init. */
inline uint64_t micros() {
  return timebase_micros();
}

/** @brief Returns monotonic time in milliseconds since init. */
inline uint64_t millis() {
  return timebase_millis();
}

/**
 * @brief Send overflow notifications to the C implementation.
 *
 * @param htim Pointer to a HAL TIM_HandleTypeDef passed to init().
 */
inline void on_period_elapsed(const void* htim) {
  timebase_on_period_elapsed(htim);
}

}
