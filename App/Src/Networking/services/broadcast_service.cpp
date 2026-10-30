#include "Networking/services/broadcast_service.hpp"

namespace Network::Services {

BroadcastService::BroadcastService(
    const char* name, const uint16_t local_port, const uint8_t rx_queue_depth,
    const uint32_t remote_ip, const uint16_t remote_port, const uint32_t interval_ms
) noexcept
    : name_(name)
    , local_port_(local_port)
    , rx_queue_depth_(rx_queue_depth)
    , remote_ip_(remote_ip)
    , remote_port_(remote_port)
    , interval_ms_(interval_ms)
{
}

} // namespace Network::Services
