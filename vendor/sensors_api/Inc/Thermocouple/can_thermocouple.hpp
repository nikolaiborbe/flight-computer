#pragma once
#include "pcan.hpp"
#include "thermocouple_codec.hpp"
#include <atomic>
#include <bitset>
#include <cstring>

namespace sensors::thermocouple {
using namespace proto::can::thermocouple;

using ThermocoupleSamples = std::array<std::pair<ThermocoupleSample, bool>, 8>;

class CanThermoCouple {
    PCan& pcan;
    pcan_node_id_t node_id;

    std::array<ThermocoupleSamples, 2> buffers{};
    std::atomic<uint8_t> read_idx{0};
    std::atomic<bool>    new_data{false};

public:
    CanThermoCouple(PCan& pcan, pcan_node_id_t node_id)
        : pcan(pcan), node_id(node_id) {}

    int  init();
    void callback(pcan_message_t* message);

    ThermocoupleSamples read_latest();

    bool has_new_data() const { return new_data.load(std::memory_order_acquire); }
};

} // namespace sensors::thermocouple