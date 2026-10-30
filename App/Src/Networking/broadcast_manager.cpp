#include "Networking/broadcast_manager.hpp"
#include "file_logger.hpp"

namespace Network {

BroadcastManager::BroadcastManager(NetworkManager& network) noexcept
    : network_(network)
{
}

bool BroadcastManager::register_service(Services::BroadcastService& service) noexcept
{
    if (service_count_ >= services_.size())
        return false;

    const int socket_id = network_.bind(service.name(), service.local_port(), service.rx_queue_depth());
    if (socket_id < 0)
        return false;

    service.bind_socket(socket_id);
    services_[service_count_++] = &service;
    return true;
}

void BroadcastManager::dispatch(const uint32_t now_ms) noexcept
{
    for (size_t i = 0; i < service_count_; ++i) {
        Services::BroadcastService& service = *services_[i];
        if (!service.due(now_ms))
            continue;

        service.mark_run(now_ms);

        uint8_t buffer[kMaxUdpPayload];
        const std::span<const uint8_t> payload = service.collect(buffer, sizeof(buffer));
        if (payload.empty())
            continue;

        if (logger_) {
            logger_->enqueue(service.local_port(), static_cast<uint16_t>(payload.size()), payload.data());
        }

        if (service.remote_ip() == 0) {
            network_.send_broadcast(service.socket_id(), service.remote_port(), payload);
        } else {
            const Ip4Endpoint dst {service.remote_ip(), service.remote_port()};
            network_.send_to(service.socket_id(), dst, payload);
        }
    }
}

} // namespace Network
