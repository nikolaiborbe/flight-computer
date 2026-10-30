#ifndef CLASS_BROADCAST_SERVICE_HPP
#define CLASS_BROADCAST_SERVICE_HPP

#include <cstddef>
#include <cstdint>
#include <span>

namespace Network::Services {

class BroadcastService {
public:
    BroadcastService(
        const char* name, uint16_t local_port, uint8_t rx_queue_depth, uint32_t remote_ip,
        uint16_t remote_port, uint32_t interval_ms
    ) noexcept;

    virtual ~BroadcastService() = default;

    BroadcastService(const BroadcastService&)            = delete;
    BroadcastService& operator=(const BroadcastService&) = delete;

    virtual std::span<const uint8_t> collect(uint8_t* buffer, size_t capacity) noexcept = 0;

    [[nodiscard]] const char* name() const noexcept { return name_; }
    [[nodiscard]] uint16_t local_port() const noexcept { return local_port_; }
    [[nodiscard]] uint8_t rx_queue_depth() const noexcept { return rx_queue_depth_; }
    [[nodiscard]] uint32_t remote_ip() const noexcept { return remote_ip_; }
    [[nodiscard]] uint16_t remote_port() const noexcept { return remote_port_; }

    [[nodiscard]] void update_interval(const uint32_t new_interval) noexcept { interval_ms_ = new_interval; }

    [[nodiscard]] bool due(uint32_t now_ms) const noexcept { return now_ms >= next_run_ms_; }
    void mark_run(uint32_t now_ms) noexcept { next_run_ms_ = now_ms + interval_ms_; }

    void bind_socket(int socket_id) noexcept { socket_id_ = socket_id; }
    [[nodiscard]] int socket_id() const noexcept { return socket_id_; }
protected:
    uint32_t interval_ms_;
private:
    const char* name_;
    uint16_t local_port_;
    uint8_t rx_queue_depth_;
    uint32_t remote_ip_;
    uint16_t remote_port_;
    uint32_t next_run_ms_ = 0;
    int socket_id_        = -1;
};

} // namespace Network::Services

#endif // CLASS_BROADCAST_SERVICE_HPP
