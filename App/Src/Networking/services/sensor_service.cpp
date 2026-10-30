#include "Networking/services/sensor_service.hpp"
#include "Data/GroundStation.hpp"
#include "Utilities/convert.hpp"
#include <cstring>
#include "Context.hpp"
namespace Network::Services {

namespace {
    constexpr uint16_t kPort = 3001;
}

SensorService::SensorService() noexcept
    : BroadcastService("Sensor", kPort, 1, 0, kPort, 101)
{
}

std::span<const uint8_t> SensorService::collect(uint8_t* buffer, size_t capacity) noexcept
{
    if (System::reduce_network_output_reader.update_and_read()) {
        update_interval(10); // ms
    }
    if (capacity < sizeof(Telemetry::GroundStation::SensorData))
        return {};

    Telemetry::GroundStation::SensorData snapshot {};

    Utils::get_current_sensordata(snapshot);

    std::memcpy(buffer, &snapshot, sizeof(snapshot));
    return std::span<const uint8_t>(buffer, sizeof(snapshot));
}

} // namespace Network::Services
