#ifndef CLASS_KALMAN_SERVICE_HPP
#define CLASS_KALMAN_SERVICE_HPP

#include "Networking/services/broadcast_service.hpp"

namespace Network::Services {

class KalmanService : public BroadcastService {
public:
    KalmanService() noexcept;

    std::span<const uint8_t> collect(uint8_t* buffer, size_t capacity) noexcept override;
};

} // namespace Network::Services

#endif // CLASS_KALMAN_SERVICE_HPP
