#pragma once

#include <pcan_core.hpp>

#include "bms_data.hpp"
#include "mutex.hpp"

namespace actuators::bms {

class CanBms {
public:
    explicit CanBms(PCan& pcan, pcan_node_id_t node_id);

    sensors::bms::RailData getRailData() const;
    sensors::bms::BatteryPackData getBatteryPackData() const;
    sensors::bms::ChargerData getChargerData() const;

private:
    void onTelemetry(pcan_message_t* msg);

    PCan& pcan_;
    pcan_node_id_t node_id_;

    mutable Mutex mutex_;
    sensors::bms::RailData rail_{};
    sensors::bms::BatteryPackData battery_{};
    sensors::bms::ChargerData charger_{};
};

}
