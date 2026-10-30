#include "adc_manager.hpp"

#ifndef STM32H723xx

namespace io::adc {

float Input::getVolts() const {
  const auto* u = reinterpret_cast<const Manager::Unit*>(unit_);
  if (!u) return 0.0f;

  const uint16_t counts = u->snap[index_];
  const uint8_t bits = u->adc_format.effective_bits;
  if (bits == 0 || bits > 16) return 0.0f;
  const float full_scale = static_cast<float>((1u << bits) - 1u);
  const float v_adc = static_cast<float>(counts) * (u->adc_format.vref / full_scale / u->adc_format.uncompensated_oversampling);

  if (u->input_transforms) {
    const auto& t = u->input_transforms[index_];
    return v_adc * t.gain + t.offset;
  }
  return v_adc;
}

uint16_t Input::getTick() const {
  const auto* u = reinterpret_cast<const Manager::Unit*>(unit_);
  if (!u) return 0;
  return u->tick;
}

bool Manager::init(std::span<const UnitConfig> units) {
  for (auto& u : units_) u = Unit{};
  n_units_ = 0;

  if (units.empty() || units.size() > kMaxUnits) return false;

  n_units_ = units.size();
  for (std::size_t i = 0; i < n_units_; ++i) {
    const auto& cfg = units[i];
    if (!cfg.hadc || !cfg.dma_buffer) return false;
    if (cfg.n_conversions == 0 || cfg.n_conversions > kMaxConvPerUnit) return false;

    auto& u = units_[i];
    u.used = true;
    u.hadc = cfg.hadc;
    u.n = cfg.n_conversions;
    u.dma = cfg.dma_buffer;
    u.adc_format = cfg.adc_format;
    u.input_transforms = cfg.input_transforms;
    u.calibrate = cfg.calibrate;
  }

  return true;
}

bool Manager::start() {
  bool ok = true;
  for (std::size_t i = 0; i < n_units_; ++i) {
    auto& u = units_[i];
    if (!u.used) continue;

    if (u.calibrate) {
#if defined(STM32H7)
      if (HAL_ADCEx_Calibration_Start(u.hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
#elif defined(STM32G4)
      if (HAL_ADCEx_Calibration_Start(u.hadc, ADC_SINGLE_ENDED) != HAL_OK) {
#endif
        ok = false;
        continue;
      }
    }

    const std::size_t len = static_cast<std::size_t>(u.n) * 2u;
    auto* buf_nv = const_cast<uint16_t*>(u.dma);

    ok &= (HAL_ADC_Start_DMA(u.hadc,
                            reinterpret_cast<uint32_t*>(buf_nv),
                            len) == HAL_OK);
  }
  return ok;
}

Manager::Unit* Manager::getUnit(const ADC_HandleTypeDef* hadc) {
  for (std::size_t i = 0; i < n_units_; ++i) {
    auto& u = units_[i];
    if (u.used && u.hadc == hadc) return &u;
  }
  return nullptr;
}

void Manager::copyHalfToSnap(Unit& u, uint8_t half) {
  const volatile uint16_t* src = u.dma + (half ? u.n : 0);
  for (uint16_t k = 0; k < u.n; ++k) {
    u.snap[k] = src[k];
  }
  ++u.tick;
}

void Manager::onDmaHalf(const ADC_HandleTypeDef* hadc) {
  if (auto* u = getUnit(hadc)) copyHalfToSnap(*u, 0);
}

void Manager::onDmaFull(const ADC_HandleTypeDef* hadc) {
  if (auto* u = getUnit(hadc)) copyHalfToSnap(*u, 1);
}

Input Manager::makeInput(uint8_t unit, uint16_t index) const {
  Input in{};
  if (unit >= n_units_) return in;
  const auto& u = units_[unit];
  if (!u.used || index >= u.n) return in;

  in.unit_ = &u;
  in.index_ = index;
  return in;
}

}

#endif // STM32H723xx
