#ifndef CLASS_SENSOR_SERVICE_HPP
#define CLASS_SENSOR_SERVICE_HPP

#include "Networking/services/broadcast_service.hpp"

namespace Network::Services {

class SensorService : public BroadcastService {
public:
    SensorService() noexcept;

    std::span<const uint8_t> collect(uint8_t* buffer, size_t capacity) noexcept override;
};

} // namespace Network::Services

#endif // CLASS_SENSOR_SERVICE_HPP
