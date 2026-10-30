#include "level_sensor.hpp"

#include "capacitance_codec.hpp"
#include "maksimilians_formulas.hpp"

namespace sensors::capacitance {

LevelSensor::LevelSensor(PCan& pcan,
                         pcan_endpoint_id_t main,
                         pcan_endpoint_id_t gas,
                         pcan_endpoint_id_t liquid)
  : pcan_(pcan) {
  pcan_.subscribe_telemetry(main, this, &LevelSensor::onMain);
  pcan_.subscribe_telemetry(gas, this, &LevelSensor::onGas);
  pcan_.subscribe_telemetry(liquid, this, &LevelSensor::onLiquid);
}

bool LevelSensor::latestPf(pcan_message_t* msg, float& out_pf) {
  CapacitanceBatch batch{};
  if (!proto::can::capacitance::decode_samples(msg->data, msg->size, batch)) {
    return false;
  }
  if (batch.count == 0) {
    return false;
  }
  out_pf = static_cast<float>(batch.samples[batch.count - 1].adjusted_cpf) / 100.0f;
  return true;
}

void LevelSensor::onMain(pcan_message_t* msg) {
  float pf;
  if (!latestPf(msg, pf)) return;
  LockGuard lock(mutex_);
  c_main_ = pf;
  have_main_ = true;
}

void LevelSensor::onGas(pcan_message_t* msg) {
  float pf;
  if (!latestPf(msg, pf)) return;
  LockGuard lock(mutex_);
  c_gas_ = pf;
  have_gas_ = true;
}

void LevelSensor::onLiquid(pcan_message_t* msg) {
  float pf;
  if (!latestPf(msg, pf)) return;
  LockGuard lock(mutex_);
  c_liquid_ = pf;
  have_liquid_ = true;
}

void LevelSensor::update(uint64_t now_us) {
  float c_main, c_gas, c_liquid;
  {
    LockGuard lock(mutex_);
    if (!have_main_ || !have_gas_ || !have_liquid_) return;
    c_main = c_main_;
    c_gas = c_gas_;
    c_liquid = c_liquid_;
  }

  const float sample = height(c_main, c_gas, c_liquid);

  constexpr uint32_t window_ms = LevelWindowSampleCount * LevelSampleSpacingMs;
  const uint32_t now_ms = static_cast<uint32_t>(now_us / 1000u);
  const uint32_t window = now_ms / window_ms;
  const std::size_t slot = (now_ms % window_ms) / LevelSampleSpacingMs;

  if (!working_valid_ || window != working_window_) {
    if (working_valid_ && working_.count > 0) {
      working_.seq = window_seq_++;
      snapshot_.publish(working_);
    }
    working_ = LevelBatch{};
    working_.dt_us = LevelSampleSpacingMs * 1000u;
    working_window_ = window;
    working_valid_ = true;
  }

  if (working_.count == 0) {
    working_.t0_us = now_us;
  }

  working_.heights_m[slot] = sample;
  working_.count = static_cast<uint8_t>(slot + 1);
}

bool LevelSensor::readLatest(LevelBatch& out) const {
  out = snapshot_.read();
  return out.count > 0;
}

}
