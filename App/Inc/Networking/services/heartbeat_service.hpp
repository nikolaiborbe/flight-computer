#ifndef CLASS_HEARTBEAT_SERVICE_HPP
#define CLASS_HEARTBEAT_SERVICE_HPP

#include "Networking/services/broadcast_service.hpp"

namespace Network::Services {

class HeartbeatService : public BroadcastService {
public:
    HeartbeatService() noexcept;

    std::span<const uint8_t> collect(uint8_t* buffer, size_t capacity) noexcept override;
};

} // namespace Network::Services

#endif // CLASS_HEARTBEAT_SERVICE_HPP
