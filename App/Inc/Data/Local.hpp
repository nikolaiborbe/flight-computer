#ifndef TELEMETRY_LOCAL_DATA_HPP
#define TELEMETRY_LOCAL_DATA_HPP

#include "Datastructures/Seqlock.hpp"
#include "GroundStation.hpp"
#include "bms_data.hpp"

namespace Telemetry::Local {

typedef struct {
    algo::Seqlock<Telemetry::GroundStation::GNSS_DATA>::Reader gnss_data;
    algo::Seqlock<Telemetry::GroundStation::BME280_DATA>::Reader bme_data;
    algo::Seqlock<Telemetry::GroundStation::MS5611_DATA>::Reader ms_data;
    algo::Seqlock<Telemetry::Shared::ImuData>::Reader icm_data;
    algo::Seqlock<Telemetry::Shared::MagnetometerData>::Reader mmc_data;
    algo::Seqlock<Telemetry::GroundStation::STIM377H_DATA>::Reader stim_data;
    algo::Seqlock<Telemetry::GroundStation::PressureData>::Reader pressure_data;
    algo::Seqlock<Telemetry::GroundStation::ThermocoupleData>::Reader thermocouple_data;

    algo::Seqlock<sensors::bms::ChargerData>::Reader charger_data;
    algo::Seqlock<sensors::bms::BatteryPackData>::Reader battery_data;
    algo::Seqlock<sensors::bms::RailData>::Reader rail_data;
} Sensors;

typedef struct {
    algo::Seqlock<Telemetry::GroundStation::KalmanData>::Reader kalman_data;
} Kalman;

// TODO make bools bitsets with each bit being an atomic operation
typedef struct {
    algo::Seqlock<bool>::Reader bme_online;
    algo::Seqlock<bool>::Reader ms_online;
    algo::Seqlock<bool>::Reader gnss_online;
    algo::Seqlock<bool>::Reader icm_online;
    algo::Seqlock<bool>::Reader mmc_online;
    algo::Seqlock<bool>::Reader stim_online;
    algo::Seqlock<bool>::Reader pressure_online;
    algo::Seqlock<bool>::Reader thermocouple_online;

    algo::Seqlock<bool>::Reader charger_data_online;
    algo::Seqlock<bool>::Reader battery_data_online;
    algo::Seqlock<bool>::Reader rail_data_online;
} SensorStatus;

typedef struct {
    algo::Seqlock<bool>::Reader separation_filling_online;
    algo::Seqlock<bool>::Reader flotation_filling_online;
    algo::Seqlock<bool>::Reader flotation_online;
    algo::Seqlock<bool>::Reader separation_servo_online;
} ActuatorStatus;

typedef struct {
    algo::Seqlock<bool>::Reader pcan1_online;
    algo::Seqlock<bool>::Reader pcan2_online;
    algo::Seqlock<bool>::Reader networking_thread_online;
    algo::Seqlock<bool>::Reader cli_thread_online;
    algo::Seqlock<bool>::Reader kalman_thread_online;
    algo::Seqlock<bool>::Reader sensor_thread_online;
    algo::Seqlock<bool>::Reader fsm_thread_online;
} HardwareStatus;

typedef struct {
    ActuatorStatus actuators;
    SensorStatus sensors;
    HardwareStatus hardware;
    algo::Seqlock<STATE_NAME>::Reader current_state;
} Status;
} // namespace Telemetry::Local

#endif
