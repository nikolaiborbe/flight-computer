#ifndef TELEMETRY_WRITERS_HPP
#define TELEMETRY_WRITERS_HPP

#include "Datastructures/Seqlock.hpp"
#include "GroundStation.hpp"
#include "bms_data.hpp"

namespace Telemetry::Writers {
struct Sensor {
    algo::Seqlock<Telemetry::Shared::ImuData>::Writer icm_data;
    algo::Seqlock<Telemetry::Shared::MagnetometerData>::Writer mmc_data;
    algo::Seqlock<Telemetry::GroundStation::STIM377H_DATA>::Writer stim_data;

    algo::Seqlock<bool>::Writer icm_online;
    algo::Seqlock<bool>::Writer mmc_online;
    algo::Seqlock<bool>::Writer stim_online;
    algo::Seqlock<bool>::Writer sensor_thread_online;
};

struct PCan {
    algo::Seqlock<Telemetry::GroundStation::GNSS_DATA>::Writer gnss_data;
    algo::Seqlock<Telemetry::GroundStation::BME280_DATA>::Writer bme_data;
    algo::Seqlock<Telemetry::GroundStation::MS5611_DATA>::Writer ms_data;
    algo::Seqlock<Telemetry::GroundStation::PressureData>::Writer pressure_data;

    algo::Seqlock<sensors::bms::BatteryPackData>::Writer batterypack_data;
    algo::Seqlock<sensors::bms::ChargerData>::Writer charger_data;
    algo::Seqlock<sensors::bms::RailData>::Writer rail_data;


    algo::Seqlock<bool>::Writer bme_online;
    algo::Seqlock<bool>::Writer gnss_online;
    algo::Seqlock<bool>::Writer ms_online;
    algo::Seqlock<bool>::Writer pressure_online;
    algo::Seqlock<bool>::Writer pcan1_online;
    algo::Seqlock<bool>::Writer pcan2_online;
    algo::Seqlock<bool>::Writer pcan_thread_online;
    algo::Seqlock<bool>::Writer thermocouple_online;

    algo::Seqlock<bool>::Writer charger_online;
    algo::Seqlock<bool>::Writer batterypack_online;
    algo::Seqlock<bool>::Writer bms_rail_online;

    algo::Seqlock<bool>::Writer separation_filling_online;
    algo::Seqlock<bool>::Writer flotation_filling_online;
    algo::Seqlock<bool>::Writer flotation_online;
    algo::Seqlock<bool>::Writer separation_servo_online;
};

struct Kalman {
    algo::Seqlock<Telemetry::GroundStation::KalmanData>::Writer kalman_data;
    algo::Seqlock<bool>::Writer kalman_thread_online;
};

struct Networking {
    algo::Seqlock<bool>::Writer network_thread_online;
};

struct Fsm {
    algo::Seqlock<bool>::Writer fsm_thread_online;
};

// Dont touch these, and dont use it
struct TestFeed {
    algo::Seqlock<Telemetry::GroundStation::GNSS_DATA>::Writer gnss_data;
    algo::Seqlock<Telemetry::GroundStation::BME280_DATA>::Writer bme_data;
    algo::Seqlock<Telemetry::GroundStation::MS5611_DATA>::Writer ms_data;
    algo::Seqlock<Telemetry::Shared::ImuData>::Writer icm_data;
    algo::Seqlock<Telemetry::GroundStation::STIM377H_DATA>::Writer stim_data;
    algo::Seqlock<Telemetry::GroundStation::PressureData>::Writer pressure_data;
};

#ifndef FC_PYTHON_TESTING
extern Sensor sensor;
extern PCan pcan;
#else
extern TestFeed test_feed;
#endif
extern Kalman kalman;
// Defined in Data/Cells.cpp once the fsm_thread_online cell is wired up there.
extern Fsm fsm;
extern Networking networking;
} // namespace Telemetry::Writers

namespace Telemetry {
// Binds every System-context reader to its telemetry cell (Data/Cells.cpp).
// Must run before any thread starts.
bool bind_readers();
} // namespace Telemetry

#endif
