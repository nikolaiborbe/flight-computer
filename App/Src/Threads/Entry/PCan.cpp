#include "Threads/Entry/PCAN.hpp"
#include "Actuators.hpp"
#include "Data/Writers.hpp"
#include "Utilities/config.h"
#include "fc_ids.h"
#include "fsm_pressure.h"

static Telemetry::Writers::PCan* pcan_writers = nullptr;

void gnss_callback(pcan_message_t* msg)
{
    if (pcan_writers == nullptr)
        return;

    Telemetry::GroundStation::GNSS_DATA gnss_data;
    size_t copy_size = (msg->size < sizeof(Telemetry::GroundStation::GNSS_DATA))
                           ? msg->size
                           : sizeof(Telemetry::GroundStation::GNSS_DATA);
    memcpy(&gnss_data, msg->data, copy_size);
    pcan_writers->gnss_online.store(gnss_data.data_valid);
    pcan_writers->gnss_data.store(gnss_data);
}

void ms5611_callback(pcan_message_t* msg)
{
    if (pcan_writers == nullptr)
        return;

    struct __attribute__((packed)) MS5611_PACKET {
        uint8_t valid;
        int32_t temperature; // Celsius * 1000
        int32_t pressure;    // mbar * 1000
        uint32_t timestamp;
    };
    static_assert(sizeof(MS5611_PACKET) == 13);

    MS5611_PACKET ms_packet = {};
    size_t copy_size = (msg->size < sizeof(MS5611_PACKET))
                           ? msg->size
                           : sizeof(MS5611_PACKET);
    memcpy(&ms_packet, msg->data, copy_size);

    Telemetry::GroundStation::MS5611_DATA ms_data = {
        .valid = ms_packet.valid,
        .temperature = static_cast<float>(ms_packet.temperature) / 1000.0f,
        .pressure = static_cast<float>(ms_packet.pressure) / 1000.0f,
        .timestamp = ms_packet.timestamp
    };
    pcan_writers->ms_online.store(ms_data.valid);
    pcan_writers->ms_data.store(ms_data);
}

void bme_callback(pcan_message_t* msg)
{
    if (pcan_writers == nullptr)
        return;

    struct __attribute__((packed)) BME280_PACKET {
        uint8_t valid;
        int32_t temperature; // Celsius * 1000
        int32_t pressure;    // hPa * 1000
        int32_t humidity;    // %RH * 1000
        uint32_t timestamp;
    };
    static_assert(sizeof(BME280_PACKET) == 17);

    BME280_PACKET bme_packet = {};
    size_t copy_size = (msg->size < sizeof(BME280_PACKET))
                           ? msg->size
                           : sizeof(BME280_PACKET);
    memcpy(&bme_packet, msg->data, copy_size);

    Telemetry::GroundStation::BME280_DATA bme_data = {
        .valid = bme_packet.valid,
        .temperature = static_cast<float>(bme_packet.temperature) / 1000.0f,
        .pressure = static_cast<float>(bme_packet.pressure) / 1000.0f,
        .humidity = static_cast<float>(bme_packet.humidity) / 1000.0f,
        .timestamp = bme_packet.timestamp
    };
    pcan_writers->bme_online.store(bme_data.valid);
    pcan_writers->bme_data.store(bme_data);
}

void bms_charger_callback(pcan_message_t* msg)
{
    if (pcan_writers == nullptr)
        return;

    // CAN FD pads payloads up to the next DLC bucket (52 -> 64 bytes here), so
    // msg->size can exceed sizeof(ChargerData); copying it unclamped smashes the stack.
    sensors::bms::ChargerData bms_charger_data;
    size_t copy_size = (msg->size < sizeof(sensors::bms::ChargerData))
                           ? msg->size
                           : sizeof(sensors::bms::ChargerData);
    memcpy(&bms_charger_data, msg->data, copy_size);
    pcan_writers->charger_online.store(bms_charger_data.faults == 0);
    pcan_writers->charger_data.store(bms_charger_data);
}

void bms_batterypack_data_callback(pcan_message_t* msg)
{
    if (pcan_writers == nullptr)
        return;
    sensors::bms::BatteryPackData battery_pack_data;
    size_t copy_size = (msg->size < sizeof(sensors::bms::BatteryPackData))
                           ? msg->size
                           : sizeof(sensors::bms::BatteryPackData);
    memcpy(&battery_pack_data, msg->data, copy_size);
    pcan_writers->batterypack_online.store(battery_pack_data.comms_ok);
    pcan_writers->batterypack_data.store(battery_pack_data);
}

void bms_rail_data_callback(pcan_message_t* msg)
{
    if (pcan_writers == nullptr)
        return;
    sensors::bms::RailData rail_data;
    size_t copy_size = (msg->size < sizeof(sensors::bms::RailData))
                           ? msg->size
                           : sizeof(sensors::bms::RailData);
    memcpy(&rail_data, msg->data, copy_size);
    pcan_writers->bms_rail_online.store(true);
    pcan_writers->rail_data.store(rail_data);
}


namespace Thread::Entry {
void Pcan(unsigned long thread_input)
{
#ifndef FC_PYTHON_TESTING
    pcan_writers = reinterpret_cast<Telemetry::Writers::PCan*>(thread_input);

    System::pcan2->subscribe_telemetry(FC_NEO_ENDPOINT, gnss_callback);
    System::pcan2->subscribe_telemetry(FC_BME280_ENDPOINT, bme_callback);
    System::pcan2->subscribe_telemetry(FC_MS5611_ENDPOINT, ms5611_callback);

    System::pcan1->subscribe_telemetry(FC_BMS_BATTERYPACK_ENDPOINT, bms_batterypack_data_callback);
    System::pcan1->subscribe_telemetry(FC_BMS_RAILDATA_ENDPOINT, bms_rail_data_callback);
    System::pcan1->subscribe_telemetry(FC_BMS_CHARGER_ENDPOINT, bms_charger_callback);

    pcan_writers->pcan1_online.store(true);
    pcan_writers->pcan2_online.store(true);
    pcan_writers->pcan_thread_online.store(true);
    while (true) {
        System::pcan1->poll();
        System::pcan2->poll();


        // betydlig forskjell
        Telemetry::GroundStation::PressureData pressure_data;
        pressure_data.flotation_pressure
            = Driver::Sensors::pressure.flotation_pressure->getPressure();
        pressure_data.separation_pressure
            = Driver::Sensors::pressure.separation_pressure->getPressure();
        pcan_writers->pressure_data.store(pressure_data);

        const bool pressure_online
            = Driver::Sensors::pressure.flotation_pressure->getFaults() == 0
              && Driver::Sensors::pressure.separation_pressure->getFaults() == 0;
        pcan_writers->pressure_online.store(pressure_online);

        // Thermocouple board not integrated yet, report online until it can be
        // derived from actual CAN traffic
        pcan_writers->thermocouple_online.store(true);

        pcan_writers->separation_servo_online.store(Driver::Actuators::servo->separation.servo.getStatus().power_state == actuators::servo::PowerState::On);;
        pcan_writers->flotation_filling_online.store(Driver::Actuators::solenoid->flotation_filling.getStatus().power_state == actuators::solenoid::PowerState::On);
        pcan_writers->separation_filling_online.store(Driver::Actuators::solenoid->separation_filling.getStatus().power_state == actuators::solenoid::PowerState::On);
        pcan_writers->flotation_online.store(Driver::Actuators::solenoid->flotation.getStatus().power_state == actuators::solenoid::PowerState::On);

        tx_thread_sleep(PCAN_THREAD_SLEEP_TIMER);
    }
    pcan_writers->pcan1_online.store(false);
    pcan_writers->pcan2_online.store(false);
    pcan_writers->pcan_thread_online.store(false);
#endif
}
} // namespace Thread::Entry
