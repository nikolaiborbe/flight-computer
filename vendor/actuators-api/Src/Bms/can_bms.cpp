#include "can_bms.hpp"
#include "ecu_ids.h"

#include <cstring>

namespace actuators::bms {

CanBms::CanBms(PCan& pcan, pcan_node_id_t node_id)
    : pcan_(pcan), node_id_(node_id) {
    pcan_.subscribe_telemetry({node_id_, PCAN_CHANNEL_ANY}, this, &CanBms::onTelemetry);
}

sensors::bms::RailData CanBms::getRailData() const {
    LockGuard lock(mutex_);
    return rail_;
}

sensors::bms::BatteryPackData CanBms::getBatteryPackData() const {
    LockGuard lock(mutex_);
    return battery_;
}

sensors::bms::ChargerData CanBms::getChargerData() const {
    LockGuard lock(mutex_);
    return charger_;
}

void CanBms::onTelemetry(pcan_message_t* msg) {
    switch (msg->channel_id) {
        case ECU_BMS_CHARGER_ENDPOINT.channel_id: {
            if (msg->size < sizeof(sensors::bms::ChargerData)) return;
            sensors::bms::ChargerData tmp{};
            std::memcpy(&tmp, msg->data, sizeof(tmp));
            LockGuard lock(mutex_);
            charger_ = tmp;
        } break;

        case ECU_BMS_BATTERY_ENDPOINT.channel_id: {
            if (msg->size < sizeof(sensors::bms::BatteryPackData)) return;
            sensors::bms::BatteryPackData tmp{};
            std::memcpy(&tmp, msg->data, sizeof(tmp));
            LockGuard lock(mutex_);
            battery_ = tmp;
        } break;

        case ECU_BMS_RAIL_ENDPOINT.channel_id: {
            if (msg->size < sizeof(sensors::bms::RailData)) return;
            sensors::bms::RailData tmp{};
            std::memcpy(&tmp, msg->data, sizeof(tmp));
            LockGuard lock(mutex_);
            rail_ = tmp;
        } break;

        default:
            break;
    }
}

}
