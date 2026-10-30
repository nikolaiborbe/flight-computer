#include "Context.hpp"
#include "Data/Writers.hpp"

namespace Cells
{
namespace
{
    algo::Seqlock<Telemetry::GroundStation::GNSS_DATA> gnss_data;
    algo::Seqlock<Telemetry::GroundStation::BME280_DATA> bme_data;
    algo::Seqlock<Telemetry::GroundStation::MS5611_DATA> ms_data;
    algo::Seqlock<Telemetry::Shared::ImuData> icm_data;
    algo::Seqlock<Telemetry::Shared::MagnetometerData> mmc_data;
    algo::Seqlock<Telemetry::GroundStation::STIM377H_DATA> stim_data;
    algo::Seqlock<Telemetry::GroundStation::PressureData> pressure_data;
    algo::Seqlock<Telemetry::GroundStation::KalmanData> kalman_data;
    algo::Seqlock<sensors::bms::BatteryPackData> batterypack_data;
    algo::Seqlock<sensors::bms::ChargerData> charger_data;
    algo::Seqlock<sensors::bms::RailData> rail_data;

    algo::Seqlock<bool> icm_online;
    algo::Seqlock<bool> mmc_online;
    algo::Seqlock<bool> stim_online;
    algo::Seqlock<bool> sensor_thread_online;

    algo::Seqlock<bool> bme_online;
    algo::Seqlock<bool> gnss_online;
    algo::Seqlock<bool> ms_online;
    algo::Seqlock<bool> pressure_online;

    algo::Seqlock<bool> thermocouple_online;
    algo::Seqlock<bool> separation_filling_online;
    algo::Seqlock<bool> flotation_filling_online;
    algo::Seqlock<bool> flotation_online;
    algo::Seqlock<bool> separation_servo_online;
    algo::Seqlock<bool> pcan1_online;
    algo::Seqlock<bool> pcan2_online;
    algo::Seqlock<bool> pcan_thread_online;
    algo::Seqlock<bool> networking_thread_online;
    algo::Seqlock<bool> kalman_thread_online;
    algo::Seqlock<bool> fsm_thread_online;

    algo::Seqlock<bool> bms_batterypack_online;
    algo::Seqlock<bool> bms_charger_online;
    algo::Seqlock<bool> bms_rail_online;
}
}

namespace Telemetry::Writers
{
#ifndef FC_PYTHON_TESTING
    Sensor sensor {
        Cells::icm_data.acquire_writer_once(),
        Cells::mmc_data.acquire_writer_once(),
        Cells::stim_data.acquire_writer_once(),
        Cells::icm_online.acquire_writer_once(),
        Cells::mmc_online.acquire_writer_once(),
        Cells::stim_online.acquire_writer_once(),
        Cells::sensor_thread_online.acquire_writer_once(),
    };

    PCan pcan {
        Cells::gnss_data.acquire_writer_once(),
        Cells::bme_data.acquire_writer_once(),
        Cells::ms_data.acquire_writer_once(),
        Cells::pressure_data.acquire_writer_once(),
        Cells::batterypack_data.acquire_writer_once(),
        Cells::charger_data.acquire_writer_once(),
        Cells::rail_data.acquire_writer_once(),
        Cells::bme_online.acquire_writer_once(),
        Cells::gnss_online.acquire_writer_once(),
        Cells::ms_online.acquire_writer_once(),
        Cells::pressure_online.acquire_writer_once(),

        Cells::pcan1_online.acquire_writer_once(),
        Cells::pcan2_online.acquire_writer_once(),
        Cells::pcan_thread_online.acquire_writer_once(),
        Cells::thermocouple_online.acquire_writer_once(),
        Cells::bms_charger_online.acquire_writer_once(),
        Cells::bms_batterypack_online.acquire_writer_once(),
        Cells::bms_rail_online.acquire_writer_once(),
        Cells::separation_filling_online.acquire_writer_once(),
        Cells::flotation_filling_online.acquire_writer_once(),
        Cells::flotation_online.acquire_writer_once(),
        Cells::separation_servo_online.acquire_writer_once(),
    };
#else
    TestFeed test_feed {
        Cells::gnss_data.acquire_writer_once(),
        Cells::bme_data.acquire_writer_once(),
        Cells::ms_data.acquire_writer_once(),
        Cells::icm_data.acquire_writer_once(),
        Cells::stim_data.acquire_writer_once(),
        Cells::pressure_data.acquire_writer_once(),
    };
#endif

    Kalman kalman {
        Cells::kalman_data.acquire_writer_once(),
        Cells::kalman_thread_online.acquire_writer_once(),
    };

    Fsm fsm {
        Cells::fsm_thread_online.acquire_writer_once()
    };

    Networking networking {
        Cells::networking_thread_online.acquire_writer_once(),
    };

}

namespace Telemetry
{
bool bind_readers()
{
    // Bind every system-context reader to its cell before any thread runs,
    // so no bind ever races a running reader or writer.
    System::data.gnss_data.bind(&Cells::gnss_data);
    System::data.bme_data.bind(&Cells::bme_data);
    System::data.ms_data.bind(&Cells::ms_data);
    System::data.icm_data.bind(&Cells::icm_data);
    System::data.mmc_data.bind(&Cells::mmc_data);
    System::data.stim_data.bind(&Cells::stim_data);
    System::data.pressure_data.bind(&Cells::pressure_data);

    System::data.battery_data.bind(&Cells::batterypack_data);
    System::data.charger_data.bind(&Cells::charger_data);
    System::data.rail_data.bind(&Cells::rail_data);

    System::status.sensors.icm_online.bind(&Cells::icm_online);
    System::status.sensors.mmc_online.bind(&Cells::mmc_online);
    System::status.sensors.stim_online.bind(&Cells::stim_online);
    System::status.sensors.bme_online.bind(&Cells::bme_online);
    System::status.sensors.gnss_online.bind(&Cells::gnss_online);
    System::status.sensors.ms_online.bind(&Cells::ms_online);
    System::status.sensors.pressure_online.bind(&Cells::pressure_online);
    System::status.hardware.sensor_thread_online.bind(&Cells::sensor_thread_online);

    System::status.sensors.battery_data_online.bind(&Cells::bms_batterypack_online);
    System::status.sensors.charger_data_online.bind(&Cells::bms_charger_online);
    System::status.sensors.rail_data_online.bind(&Cells::bms_rail_online);

    System::kalman_data.bind(&Cells::kalman_data);
    System::status.sensors.thermocouple_online.bind(&Cells::thermocouple_online);

      System::status.actuators.separation_filling_online.bind(&Cells::separation_filling_online);
      System::status.actuators.flotation_filling_online.bind(&Cells::flotation_filling_online);
      System::status.actuators.flotation_online.bind(&Cells::flotation_online);
      System::status.actuators.separation_servo_online.bind(&Cells::separation_servo_online);

      System::status.hardware.pcan1_online.bind(&Cells::pcan1_online);
      System::status.hardware.pcan2_online.bind(&Cells::pcan2_online);
      System::status.hardware.networking_thread_online.bind(&Cells::networking_thread_online);
      System::status.hardware.kalman_thread_online.bind(&Cells::kalman_thread_online);
      System::status.hardware.fsm_thread_online.bind(&Cells::fsm_thread_online);

    System::reduce_network_output_writer = System::reduce_network_seqlock.acquire_writer_once();
    System::reduce_network_output_reader.bind(&System::reduce_network_seqlock);

    System::ground_station_heartbeat_writer = System::ground_station_heartbeat_seqlock.acquire_writer_once();
    System::ground_station_heartbeat.bind(&System::ground_station_heartbeat_seqlock);
    return true;
}
} // namespace Telemetry
