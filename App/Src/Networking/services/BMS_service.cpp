#include "services/BMS_service.hpp"
#include "Context.hpp"
#include "Data/GroundStation.hpp"
#include "Networking/services/sensor_service.hpp"
#include "Utilities/convert.hpp"
#include "bms_data.hpp"
#include <cstddef>
#include <cstring>
namespace Network::Services {

namespace {
constexpr uint16_t kPort = 6967;

// The Telemetry::GroundStation wire structs are duplicated from
// vendor/actuators-api/Inc/Bms/bms_data.hpp (the groundstation tooling parses
// GroundStation.hpp, so they can't just alias the vendor types). Enforce that
// the copies never drift from the vendor layout.
#define BMS_ASSERT_FIELD(APP, VENDOR, FIELD)                                                       \
    static_assert(offsetof(APP, FIELD) == offsetof(VENDOR, FIELD)                                  \
                      && sizeof(APP::FIELD) == sizeof(VENDOR::FIELD),                              \
                  #APP "::" #FIELD " diverged from vendor bms_data.hpp")

static_assert(sizeof(::BatteryPackData) == sizeof(sensors::bms::BatteryPackData));
BMS_ASSERT_FIELD(::BatteryPackData, sensors::bms::BatteryPackData, cell);
BMS_ASSERT_FIELD(::BatteryPackData, sensors::bms::BatteryPackData, voltage);
BMS_ASSERT_FIELD(::BatteryPackData, sensors::bms::BatteryPackData, current);
BMS_ASSERT_FIELD(::BatteryPackData, sensors::bms::BatteryPackData, die_temperature);
BMS_ASSERT_FIELD(::BatteryPackData, sensors::bms::BatteryPackData, balancing_mask);
BMS_ASSERT_FIELD(::BatteryPackData, sensors::bms::BatteryPackData, alert);
BMS_ASSERT_FIELD(::BatteryPackData, sensors::bms::BatteryPackData, comms_ok);

static_assert(sizeof(::ChargerData) == sizeof(sensors::bms::ChargerData));
BMS_ASSERT_FIELD(::ChargerData, sensors::bms::ChargerData, vac);
BMS_ASSERT_FIELD(::ChargerData, sensors::bms::ChargerData, vbat);
BMS_ASSERT_FIELD(::ChargerData, sensors::bms::ChargerData, vsys);
BMS_ASSERT_FIELD(::ChargerData, sensors::bms::ChargerData, iac);
BMS_ASSERT_FIELD(::ChargerData, sensors::bms::ChargerData, ibat);
BMS_ASSERT_FIELD(::ChargerData, sensors::bms::ChargerData, temperature);
BMS_ASSERT_FIELD(::ChargerData, sensors::bms::ChargerData, charging);
BMS_ASSERT_FIELD(::ChargerData, sensors::bms::ChargerData, power_good);
BMS_ASSERT_FIELD(::ChargerData, sensors::bms::ChargerData, faults);
BMS_ASSERT_FIELD(::ChargerData, sensors::bms::ChargerData, state);

static_assert(sizeof(::RailMeasurement) == sizeof(sensors::bms::RailMeasurement));
BMS_ASSERT_FIELD(::RailMeasurement, sensors::bms::RailMeasurement, current);
BMS_ASSERT_FIELD(::RailMeasurement, sensors::bms::RailMeasurement, volts);
BMS_ASSERT_FIELD(::RailMeasurement, sensors::bms::RailMeasurement, shunt_voltage);
BMS_ASSERT_FIELD(::RailMeasurement, sensors::bms::RailMeasurement, power);

static_assert(sizeof(::RailData) == sizeof(sensors::bms::RailData));
BMS_ASSERT_FIELD(::RailData, sensors::bms::RailData, vbat);
BMS_ASSERT_FIELD(::RailData, sensors::bms::RailData, v_12);
BMS_ASSERT_FIELD(::RailData, sensors::bms::RailData, v_24);

static_assert(static_cast<uint8_t>(::ChargingStatus::NotCharging)
                  == static_cast<uint8_t>(sensors::bms::ChargingStatus::NotCharging)
              && static_cast<uint8_t>(::ChargingStatus::ChargeTermination)
                  == static_cast<uint8_t>(sensors::bms::ChargingStatus::ChargeTermination));

#undef BMS_ASSERT_FIELD
}

BMSService::BMSService() noexcept
    : BroadcastService("BMS", kPort, 1, 0, kPort, 881)
{
}

std::span<const uint8_t> BMSService::collect(uint8_t* buffer, size_t capacity) noexcept
{
    if (System::reduce_network_output_reader.update_and_read()) {
        update_interval(250); // ms
    }

    if (capacity < sizeof(Telemetry::GroundStation::BMS_Data))
        return {};

    Telemetry::GroundStation::BMS_Data snapshot {};

    const auto battery = System::data.battery_data.update_and_read();
    const auto charger = System::data.charger_data.update_and_read();
    const auto rail = System::data.rail_data.update_and_read();

    // The app and vendor structs share an identical layout (enforced by the
    // static_asserts above), so each sub-struct copies with a single memcpy.
    std::memcpy(&snapshot.battery, &battery, sizeof(snapshot.battery));
    std::memcpy(&snapshot.charger, &charger, sizeof(snapshot.charger));
    std::memcpy(&snapshot.rail, &rail, sizeof(snapshot.rail));

    std::memcpy(buffer, &snapshot, sizeof(snapshot));
    return std::span<const uint8_t>(buffer, sizeof(snapshot));
}

} // namespace Network::Services
