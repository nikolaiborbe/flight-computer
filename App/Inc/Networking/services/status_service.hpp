#ifndef CLASS_STATUS_SERVICE_HPP
#define CLASS_STATUS_SERVICE_HPP

#include "Networking/services/broadcast_service.hpp"

namespace Network::Services {

class StatusService : public BroadcastService {
public:
    StatusService() noexcept;

    std::span<const uint8_t> collect(uint8_t* buffer, size_t capacity) noexcept override;
};

} // namespace Network::Services

#endif // CLASS_STATUS_SERVICE_HPP
