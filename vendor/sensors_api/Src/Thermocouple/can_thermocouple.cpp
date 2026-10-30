#include "can_thermocouple.hpp"
#include "thermocouple_codec.hpp"

namespace sensors::thermocouple {

int CanThermoCouple::init() {
    pcan.subscribe_telemetry({node_id, PCAN_CHANNEL_ANY}, this, &CanThermoCouple::callback);
    return 0;
}

void CanThermoCouple::callback(pcan_message_t* message) {
    if (message->channel_id > 4) return;

    uint8_t next = 1 - read_idx.load(std::memory_order_relaxed);

    ThermocoupleSample sample;
    std::memcpy(&sample, message->data, sizeof(ThermocoupleSample));
    
    ThermocoupleSamples samples = buffers[1 - next];

    size_t index = message->channel_id;
    if (index >= samples.size()) return;

    samples[index] = std::pair(sample, true);
    buffers[next] = samples;

    read_idx.store(next, std::memory_order_release);

    new_data.store(true, std::memory_order_release);
}

ThermocoupleSamples CanThermoCouple::read_latest() {
    uint8_t idx = read_idx.load(std::memory_order_acquire);

    new_data.store(false, std::memory_order_relaxed);

    return buffers[idx];
}

} // namespace sensors::thermocouple