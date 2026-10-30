#ifndef CLASS_BROADCAST_MANAGER_HPP
#define CLASS_BROADCAST_MANAGER_HPP

#include "Networking/network_manager.hpp"
#include "Networking/services/broadcast_service.hpp"
#include "file_logger.hpp"
#include <array>
#include <cstddef>

namespace Network {

class BroadcastManager {
public:
    static constexpr size_t kMaxServices = 8;

    explicit BroadcastManager(NetworkManager& network) noexcept;

    bool register_service(Services::BroadcastService& service) noexcept;

    void attach_logger(FileLogger& logger) noexcept { logger_ = &logger; }

    void dispatch(uint32_t now_ms) noexcept;

private:
    NetworkManager& network_;
    std::array<Services::BroadcastService*, kMaxServices> services_ {};
    size_t service_count_ = 0;
    FileLogger* logger_   = nullptr;
};

} // namespace Network

#endif
