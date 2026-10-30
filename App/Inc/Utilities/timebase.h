/**
 * @file timebase.h
 *
 * @brief Monotonic timebase (C API).
 *
 * This module provides a 64-bit monotonic microsecond clock based on a
 * hardware timer (e.g. TIM5 configured at 1 MHz).
 *
 * The implementation extends a 32-bit timer counter to 64-bit by tracking
 * timer overflows from the timer update interrupt. The API is safe to call
 * from any thread at any time.
 *
 * This header is C-compatible and intended to be included from both C and C++
 * translation units. For a C++ convenience wrapper, include @ref timebase.hpp instead.
 *
 * @note Important:
 *  - The timer must run continuously and tick at 1 MHz.
 *  - The timer update interrupt must be enabled in NVIC and must call
 *    timebase_on_period_elapsed() from HAL_TIM_PeriodElapsedCallback().
 *
 * @copyright Propulse NTNU (c) 2026
 */

#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief Initialize the timebase using the given timer instance.
  *
  * Call once during system bring-up (after MX_TIMx_Init()) before any threads start.
  *
  * @param htim Pointer to the HAL timer handle (e.g. &htim5).
  */
void timebase_init(void* htim);

/**
 * @brief Get monotonic time in microseconds since timebase_init().
 *
 * The returned value is 64-bit and monotonically increasing for the lifetime
 * of the system.
 *
 * @return Time in microseconds.
 */
uint64_t timebase_micros(void);

/**
 * @brief Get monotonic time in milliseconds since timebase_init().
 *
 * @return Time in milliseconds.
 */
uint64_t timebase_millis(void);

/**
 * @brief Notify the timebase of a timer update (overflow) event.
 *
 * Must be called from HAL_TIM_PeriodElapsedCallback() to extend the underlying
 * hardware timer counter to 64-bit.
 *
 * @param htim Pointer to the HAL timer handle passed to timebase_init().
 */
void timebase_on_period_elapsed(const void* htim);

#ifdef __cplusplus
}
#endif
