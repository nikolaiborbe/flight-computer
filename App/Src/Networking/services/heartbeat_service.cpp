#include "Networking/services/heartbeat_service.hpp"
#include "Context.hpp"
#include <cstring>

namespace Network::Services {

namespace {
    constexpr char kMessage[] = "I'm alive!";
    constexpr uint16_t kLocalPort  = 0xDADA;
    constexpr uint16_t kRemotePort = 0xDADA;
}

HeartbeatService::HeartbeatService() noexcept
    : BroadcastService("Heartbeat", kLocalPort, 1, 0, kRemotePort, 977)
{
}

std::span<const uint8_t> HeartbeatService::collect(uint8_t* buffer, size_t capacity) noexcept
{
    if (System::reduce_network_output_reader.update_and_read()) {
        update_interval(1000); // ms
    }
    const size_t len = sizeof(kMessage) < capacity ? sizeof(kMessage) : capacity;
    std::memcpy(buffer, kMessage, len);
    return std::span<const uint8_t>(buffer, len);
}

} // namespace Network::Services
