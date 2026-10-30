#pragma once

#include <nx_api.h>
#include <array>
#include <cstdint>
#include <span>
#include <type_traits>

namespace Network {

inline constexpr size_t kMaxUdpPayload = 256;
inline constexpr size_t kMaxSockets    = 8;
inline constexpr uint8_t kRxDepthMax   = 8;

static constexpr ULONG kIpv4Broadcast = 0xFFFFFFFFu;

struct Ip4Endpoint {
    uint32_t address = 0;
    uint16_t port    = 0;
};

struct RxPacket {
    int socket_id = -1;
    Ip4Endpoint src {};
    ULONG rx_tick = 0;
    uint16_t len  = 0;
    uint8_t data[kMaxUdpPayload] {};

    operator std::span<const uint8_t>() const { return std::span<const uint8_t>(data, len); }
};

class NetworkManager {

public:
    NetworkManager(NX_IP& nx_ip, NX_PACKET_POOL& nx_packet_pool) noexcept;

    UINT start() noexcept;

    void poll() noexcept;

    int bind(const char* name, uint16_t local_port, uint8_t rx_queue_depth) noexcept;

    bool send_to(int socket_id, const Ip4Endpoint& dst, std::span<const uint8_t> data) noexcept;

    bool send_broadcast(int socket_id, uint16_t port, std::span<const uint8_t> data) noexcept;

    UINT receive(int socket_id, RxPacket& packet, ULONG wait_option) noexcept;

private:
    struct Binding {
        NX_UDP_SOCKET socket {};
        bool bound = false;
    };

    std::array<Binding, kMaxSockets> bindings_ {};
    size_t binding_count_ = 0;

    NX_IP& nx_ip_;
    NX_PACKET_POOL& nx_packet_pool_;

    struct TxJob {
        NX_PACKET* nx_packet = nullptr;
        ULONG dst_ip         = 0;
        UINT dst_port        = 0;
        int socket_id        = -1;
    };
    static_assert(std::is_trivially_copyable_v<TxJob>);

    static constexpr ULONG kTxJobULongs = (sizeof(TxJob) + sizeof(ULONG) - 1) / sizeof(ULONG);

    static constexpr size_t kTxDepth = 32;

    struct TxQueue {
        TX_QUEUE queue {};
        alignas(ULONG) std::array<ULONG, kTxDepth * kTxJobULongs> tx_buffer {};
    };

    TxQueue tx_queue_ {};

    bool enqueue_tx(const TxJob& tx_job) noexcept;

    struct RxItem {
        NX_PACKET* nx_packet = nullptr;
        ULONG rx_tick        = 0;
    };

    static constexpr ULONG kRxItemULongs = (sizeof(RxItem) + sizeof(ULONG) - 1) / sizeof(ULONG);

    struct RxQueue {
        TX_QUEUE queue {};
        alignas(ULONG) std::array<ULONG, kRxDepthMax * kRxItemULongs> rx_buffer {};
    };

    std::array<RxQueue, kMaxSockets> rx_queues_ {};

    static UINT
    copy_payload(NX_PACKET* nx_packet, uint8_t* dst, uint16_t dst_cap, uint16_t& out_len) noexcept;

    static CHAR* tx_name(const char* s) noexcept
    {
        return const_cast<CHAR*>(s); // threadx is too extra sometimes
    }
};

} // namespace Network
