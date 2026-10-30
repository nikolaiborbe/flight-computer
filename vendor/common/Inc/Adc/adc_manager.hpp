#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

extern "C" {
#include "adc.h"
}

namespace io::adc {

struct InputTransform {
  float gain = 1.0f;
  float offset = 0.0f;
};

struct AdcFormat {
  float vref = 3.3f;
  uint8_t effective_bits = 16;
  uint8_t uncompensated_oversampling = 1;
};

class Input {
public:
  float getVolts() const;

  uint16_t getTick() const;

  explicit operator bool() const { return unit_ != nullptr; }

private:
  friend class Manager;
  const void* unit_ = nullptr;
  uint16_t index_ = 0;
};

class Manager {
public:
  static constexpr std::size_t kMaxUnits = 3;
  static constexpr std::size_t kMaxConvPerUnit = 16;

  struct UnitConfig {
    ADC_HandleTypeDef* hadc = nullptr;
    uint16_t n_conversions = 0;
    volatile uint16_t* dma_buffer = nullptr;
    AdcFormat adc_format{};
    const InputTransform* input_transforms = nullptr;
    bool calibrate = true;
  };

  bool init(std::span<const UnitConfig> units);
  bool start();

  void onDmaHalf(const ADC_HandleTypeDef* hadc);
  void onDmaFull(const ADC_HandleTypeDef* hadc);

  Input makeInput(uint8_t unit, uint16_t index) const;

private:
  struct Unit {
    bool used = false;
    uint16_t n = 0;
    ADC_HandleTypeDef* hadc = nullptr;
    volatile uint16_t* dma = nullptr;
    AdcFormat adc_format{};
    const InputTransform* input_transforms = nullptr;
    bool calibrate = true;

    alignas(4) std::array<volatile uint16_t, kMaxConvPerUnit> snap{};
    volatile uint16_t tick = 0;
  };

  Unit* getUnit(const ADC_HandleTypeDef* hadc);
  void copyHalfToSnap(Unit& u, uint8_t half);

  std::array<Unit, kMaxUnits> units_{};
  std::size_t n_units_ = 0;

  friend class Input;
};

}
