/*#include "can_load_cell.hpp"

#include "loadcell_codec.hpp"
#include "timebase.hpp"
#include "units.hpp"

namespace sensors::loadcell {

CanLoadCell::CanLoadCell(PCan& pcan, pcan_endpoint_id_t address)
    : pcan_(pcan),
      address_(address) {
  State s{};
  s.load_kg = 0.0f;
  s.fault = Fault::kCommFault;
  s.last_rx_us = 0;
  s.last_batch_seq = 0;
  s.batch_seq_valid = false;
  s.rx_drops = 0;
  state_snapshot_.publish(s);

  pcan_.subscribe_telemetry(address_, this, &CanLoadCell::onTelemetry);
}

float CanLoadCell::getWeight() const {
  return state_snapshot_.read().load_kg;
}

Fault CanLoadCell::getFault() const {
  return state_snapshot_.read().fault;
}

const Config& CanLoadCell::getConfig() const {
  return config_;
}

bool CanLoadCell::isStale(uint64_t now_us) const {
  const State s = state_snapshot_.read();
  return (now_us - s.last_rx_us) > kStaleTimeoutUs;
}

void CanLoadCell::onTelemetry(pcan_message_t* msg) {
  if (!msg || msg->size < 1) {
    return;
  }

  const uint64_t now_us = timebase::micros();
  const auto type = static_cast<proto::can::loadcell::FrameType>(msg->data[0]);

  switch (type) {
    case proto::can::loadcell::FrameType::kStatus:
      (void)handleStatus(now_us, msg->data, msg->size);
      break;

    case proto::can::loadcell::FrameType::kSamples:
      (void)handleSamples(now_us, msg->data, msg->size);
      break;

    default:
      break;
  }
}

bool CanLoadCell::handleStatus(uint64_t now_us,
                                     const uint8_t* payload,
                                     std::size_t len) {
  proto::can::loadcell::StatusFrame st{};
  if (!proto::can::loadcell::decode_status(payload, len, st)) {
    return false;
  }

  State s = state_snapshot_.read();
  s.fault = st.fault;
  s.last_rx_us = now_us;
  state_snapshot_.publish(s);

  return true;
}

bool CanLoadCell::handleSamples(uint64_t now_us,
                                      const uint8_t* payload,
                                      std::size_t len) {
  LoadCellBatch batch{};
  if (!proto::can::loadcell::decode_samples(payload, len, batch)) {
    return false;
  }

  State s = state_snapshot_.read();

  if (s.batch_seq_valid) {
    const uint8_t expected = static_cast<uint8_t>(s.last_batch_seq + 1u);
    if (batch.seq != expected) {
      ++s.rx_drops;
      batch.overrun = true;
    }
  }

  s.batch_seq_valid = true;
  s.last_batch_seq = batch.seq;
  s.last_rx_us = now_us;

  if (batch.count > 0 && batch.count <= LoadCellBatchSamples) {
  	s.load_kg = units::dkg_to_kg(batch.samples[batch.count - 1]);
  }

  if (s.fault == Fault::kCommFault || s.fault == Fault::kStale) {
    s.fault = Fault::kNone;
  }

  state_snapshot_.publish(s);
  batch_buffer_.pushItem(batch);

  return true;
}

}
*/