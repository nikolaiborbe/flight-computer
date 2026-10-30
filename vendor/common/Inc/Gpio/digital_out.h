#pragma once

extern "C" {
#if defined(STM32G473xx) || defined(STM32G431xx)
#include "stm32g4xx_hal.h"
#elif defined(STM32H753xx) || defined(STM32H723xx)
#include "stm32h7xx_hal.h"
#endif
}

class DigitalOut {
public:
  DigitalOut(GPIO_TypeDef* port, uint16_t pin)
    : m_port(port), m_pin(pin) {}

  void set(const bool high) {
    HAL_GPIO_WritePin(m_port, m_pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }

private:
  GPIO_TypeDef* m_port;
  uint16_t m_pin;
};
