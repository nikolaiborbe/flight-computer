#include "Networking/services/status_service.hpp"
#include "Context.hpp"
#include <cstring>

namespace Network::Services {

namespace {
constexpr uint16_t kPort = 3003;
}

StatusService::StatusService() noexcept
    : BroadcastService("Status", kPort, 1, 0, kPort, 937)
{
}

std::span<const uint8_t> StatusService::collect(uint8_t* buffer, size_t capacity) noexcept
{
    if (System::reduce_network_output_reader.update_and_read()) {
        update_interval(1000);
    }

    if (capacity < sizeof(Telemetry::GroundStation::SystemStatus))
        return {};

    Telemetry::GroundStation::SystemStatus snapshot {};

    snapshot.current_state = System::current_state.update_and_read();

    Telemetry::Local::SensorStatus& sensors = System::status.sensors;
    snapshot.sensors.bme_online             = System::status.sensors.bme_online.update_and_read();
    snapshot.sensors.ms_online              = System::status.sensors.ms_online.update_and_read();
    snapshot.sensors.gnss_online            = System::status.sensors.gnss_online.update_and_read();
    snapshot.sensors.icm_online             = System::status.sensors.icm_online.update_and_read();
    snapshot.sensors.mmc_online             = System::status.sensors.mmc_online.update_and_read();
    snapshot.sensors.stim_online            = System::status.sensors.stim_online.update_and_read();
    snapshot.sensors.pressure_online = System::status.sensors.pressure_online.update_and_read();

    Telemetry::Local::ActuatorStatus& actuators = System::status.actuators;
    snapshot.actuators.separation_filling_online
        = System::status.actuators.separation_filling_online.update_and_read();
    snapshot.actuators.flotation_filling_online
        = System::status.actuators.flotation_filling_online.update_and_read();
    snapshot.actuators.flotation_online
        = System::status.actuators.flotation_online.update_and_read();
    snapshot.actuators.separation_servo_online
        = System::status.actuators.separation_servo_online.update_and_read();

    Telemetry::Local::HardwareStatus& hardware = System::status.hardware;
    snapshot.hardware.pcan1_online = System::status.hardware.pcan1_online.update_and_read();
    snapshot.hardware.pcan2_online = System::status.hardware.pcan2_online.update_and_read();
    snapshot.hardware.networking_thread_online
        = System::status.hardware.networking_thread_online.update_and_read();
    snapshot.hardware.kalman_thread_online
        = System::status.hardware.kalman_thread_online.update_and_read();
    snapshot.hardware.sensor_thread_online
        = System::status.hardware.sensor_thread_online.update_and_read();
    snapshot.hardware.fsm_thread_online
        = System::status.hardware.fsm_thread_online.update_and_read();

    std::memcpy(buffer, &snapshot, sizeof(snapshot));
    return std::span<const uint8_t>(buffer, sizeof(snapshot));
}

} // namespace Network::Services
