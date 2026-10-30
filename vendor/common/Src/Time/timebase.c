/**
 * @file timebase.c
 *
 * @brief Monotonic timebase implementation.
 *
 * This implementation builds a 64-bit monotonic microsecond timestamp from a
 * hardware timer configured at 1 MHz.
 *
 * Implementation details:
 *  - The timer counter provides the lower 32 bits (microsecond ticks).
 *  - A software overflow counter (s_hi) provides the upper 32 bits and is
 *    incremented from the timer update interrupt (HAL_TIM_PeriodElapsedCallback).
 *  - Safe to call from any thread at any time.
 *
 * @copyright Propulse NTNU (c) 2026
 */

#include "timebase.h"

#include "tim.h"

/** @brief Selected timer handle passed at init time. */
static TIM_HandleTypeDef* s_htim = NULL;

/** @brief Upper 32 bits of the extended timer counter (overflow count). */
static volatile uint32_t s_hi = 0;

void timebase_init(void* htim) {
  s_htim = htim;
  s_hi = 0;

  // Enable update interrupt and start the timer in interrupt mode.
  __HAL_TIM_CLEAR_FLAG(s_htim, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(s_htim, TIM_IT_UPDATE);
  HAL_TIM_Base_Start_IT(s_htim);
}

void timebase_on_period_elapsed(const void* htim) {
  if (htim == s_htim) {
    s_hi++;
  }
}

uint64_t timebase_micros(void) {
  if (s_htim == NULL) return 0;

  const uint32_t priMask = __get_PRIMASK();
  __disable_irq();

  uint32_t hi = s_hi;
  uint32_t lo = __HAL_TIM_GET_COUNTER(s_htim);

  // Account for an overflow that is pending but whose ISR has not run yet.
  // Do not clear the flag here; HAL_TIM_IRQHandler() must still consume it
  // and commit the increment to s_hi.
  if (__HAL_TIM_GET_FLAG(s_htim, TIM_FLAG_UPDATE) != RESET) {
    hi++;
    lo = __HAL_TIM_GET_COUNTER(s_htim);
  }

  if (!priMask) __enable_irq();

  return (uint64_t) hi << 32 | lo;
}

uint64_t timebase_millis(void) {
  return timebase_micros() / 1000;
}
