#include "Networking/services/kalman_service.hpp"
#include "Context.hpp"
#include "Data/GroundStation.hpp"
#include <cstring>

namespace Network::Services {

namespace {
    constexpr uint16_t kPort = 3002;
}

KalmanService::KalmanService() noexcept
    : BroadcastService("Kalman", kPort, 1, 0, kPort, 100)
{
}

std::span<const uint8_t> KalmanService::collect(uint8_t* buffer, size_t capacity) noexcept
{
    if (System::reduce_network_output_reader.update_and_read()) {
        update_interval(100); // ms
    }

    if (capacity < sizeof(Telemetry::GroundStation::KalmanData))
        return {};

    const Telemetry::GroundStation::KalmanData& test = System::kalman_data.update_and_read();
    Telemetry::GroundStation::KalmanData snapshot = test;
    
    std::memcpy(buffer, &snapshot, sizeof(snapshot));
    return std::span<const uint8_t>(buffer, sizeof(snapshot));
}

} // namespace Network::Services
