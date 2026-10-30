#pragma once

#include <cstdint>

class PwmChannel {
public:
  PwmChannel(TIM_HandleTypeDef* timer, uint32_t channel)
    : timer_(timer), channel_(channel) {}

  bool valid() const {
    return timer_ != nullptr && timer_->Instance != nullptr;
  }

  bool start() {
    if (!valid()) return false;
    return HAL_TIM_PWM_Start(timer_, channel_) == HAL_OK;
  }

  bool stop() {
    if (!valid()) return false;
    return HAL_TIM_PWM_Stop(timer_, channel_) == HAL_OK;
  }

  bool setCompare(uint16_t ticks) {
    if (!valid()) return false;
    __HAL_TIM_SetCompare(timer_, channel_, ticks);
    return true;
  }

  float ticksPerUs() const {
    if (!valid()) return 0.0f;

    const uint32_t timer_clk_hz = timerClockHz();
    if (timer_clk_hz == 0u) return 0.0f;

    const uint32_t prescaler = timer_->Instance->PSC;
    const float tick_hz =
        static_cast<float>(timer_clk_hz) / static_cast<float>(prescaler + 1u);

    return tick_hz / 1'000'000.0f;
  }

private:
#if defined(STM32G473xx) || defined(STM32G431xx)
  uint32_t timerClockHz() const  {
    if (!valid()) return 0u;

    const bool is_apb2_timer =
        (timer_->Instance == TIM1)  || (timer_->Instance == TIM8)  ||
        (timer_->Instance == TIM15) || (timer_->Instance == TIM16) ||
        (timer_->Instance == TIM17);

    // PCLK1 or PCLK2
    const uint32_t pclk = is_apb2_timer ? HAL_RCC_GetPCLK2Freq()
                                        : HAL_RCC_GetPCLK1Freq();

    // If APB prescaler != 1, timer clock = 2 * PCLK (on STM32G4)
    const uint32_t apb_div =
        is_apb2_timer ? HAL_RCC_GetPCLK2Freq() == 0 ? 1u : (HAL_RCC_GetHCLKFreq() / HAL_RCC_GetPCLK2Freq())
                      : HAL_RCC_GetPCLK1Freq() == 0 ? 1u : (HAL_RCC_GetHCLKFreq() / HAL_RCC_GetPCLK1Freq());

    // apb_div can be 1,2,4,8,16. Timer clock doubles when apb_div > 1.
    return (apb_div == 1u) ? pclk : (pclk * 2u);
  }
#elif defined(STM32H753xx) || defined(STM32H723xx)
  uint32_t timerClockHz() const {
    // If you ever call this before RCC configured, it still shouldn't hard fault,
    // because it only reads RCC registers; but keep valid() guard above anyway.
    uint32_t pclk;
    uint32_t ppre_bits;
    bool apb_presc_is_div1;

    const bool is_apb2_timer =
        (timer_->Instance == TIM1) || (timer_->Instance == TIM8) ||
        (timer_->Instance == TIM15) || (timer_->Instance == TIM16) ||
        (timer_->Instance == TIM17);

    if (is_apb2_timer) {
      pclk = HAL_RCC_GetPCLK2Freq();
      ppre_bits = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE2);
      apb_presc_is_div1 = (ppre_bits == RCC_D2CFGR_D2PPRE2_DIV1);
    } else {
      pclk = HAL_RCC_GetPCLK1Freq();
      ppre_bits = (RCC->D2CFGR & RCC_D2CFGR_D2PPRE1);
      apb_presc_is_div1 = (ppre_bits == RCC_D2CFGR_D2PPRE1_DIV1);
    }

    return apb_presc_is_div1 ? pclk : (pclk * 2u);
  }
#endif
  TIM_HandleTypeDef* timer_ = nullptr;
  uint32_t channel_ = 0;
};
