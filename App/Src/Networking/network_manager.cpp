#include "Networking/network_manager.hpp"
#include "tx_api.h"
#include "nx_ip.h"
#include "nx_udp.h"
#include "app_netxduo.h"

extern NX_IP NetXDuoEthIpInstance;

namespace Network {

NetworkManager::NetworkManager(NX_IP& nx_ip, NX_PACKET_POOL& nx_packet_pool) noexcept
    : nx_ip_(nx_ip)
    , nx_packet_pool_(nx_packet_pool)
{
}

UINT NetworkManager::start() noexcept
{
    const ULONG tx_bytes = kTxDepth * kTxJobULongs * sizeof(ULONG);
    const UINT status    = tx_queue_create(
        &tx_queue_.queue, tx_name("NET_TX"), kTxJobULongs, tx_queue_.tx_buffer.data(), tx_bytes
    );

    return status;
}

int NetworkManager::bind(const char* name, const uint16_t local_port, uint8_t rx_queue_depth) noexcept
{
    if (binding_count_ >= bindings_.size())
        return -1;

    const size_t i = binding_count_;

    if (rx_queue_depth == 0)
        rx_queue_depth = 1;
    if (rx_queue_depth > kRxDepthMax)
        rx_queue_depth = kRxDepthMax;

    const ULONG rx_bytes = static_cast<ULONG>(rx_queue_depth) * kRxItemULongs * sizeof(ULONG);
    UINT status          = tx_queue_create(
        &rx_queues_[i].queue, tx_name(name), kRxItemULongs, rx_queues_[i].rx_buffer.data(), rx_bytes
    );
    if (status != TX_SUCCESS)
        return -1;

    auto& binding = bindings_[i];
    status        = nx_udp_socket_create(
        &nx_ip_, &binding.socket, tx_name(name), NX_IP_NORMAL, NX_FRAGMENT_OKAY, NX_IP_TIME_TO_LIVE,
        kMaxUdpPayload
    );
    if (status != NX_SUCCESS)
        return -1;

    status = nx_udp_socket_bind(&binding.socket, local_port, NX_NO_WAIT);
    if (status != NX_SUCCESS)
        return -1;

    binding.bound  = true;
    binding_count_ = i + 1;

    return static_cast<int>(i);
}

void NetworkManager::poll() noexcept
{
    // Receive
    for (size_t i = 0; i < binding_count_; i++) {
        auto& binding = bindings_[i];
        if (!binding.bound)
            continue;

        while (true) {
            NX_PACKET* nx_packet = nullptr;
            const UINT status    = nx_udp_socket_receive(&binding.socket, &nx_packet, NX_NO_WAIT);
            if (status != NX_SUCCESS) {
                break; // No more packets to receive
            }

            RxItem rx_item {};
            rx_item.nx_packet = nx_packet;
            rx_item.rx_tick   = tx_time_get();

            const UINT q_status = tx_queue_send(&rx_queues_[i].queue, &rx_item, TX_NO_WAIT);
            if (q_status != TX_SUCCESS) {
                nx_packet_release(nx_packet); // Queue full so avoid leaking the packet pool
            }
        }
    }

    // Send
    while (true) {
        TxJob tx_job {};
        const UINT qst = tx_queue_receive(&tx_queue_.queue, &tx_job, TX_NO_WAIT);
        if (qst != TX_SUCCESS)
            break;

        if (tx_job.socket_id < 0 || static_cast<size_t>(tx_job.socket_id) >= binding_count_) {
            nx_packet_release(tx_job.nx_packet);
            continue;
        }

        auto& binding = bindings_[tx_job.socket_id];
        if (!binding.bound) {
            nx_packet_release(tx_job.nx_packet);
            continue;
        }

        const UINT st
            = nx_udp_socket_send(&binding.socket, tx_job.nx_packet, tx_job.dst_ip, tx_job.dst_port);
        if (st != NX_SUCCESS) {
            nx_packet_release(tx_job.nx_packet);
        }
    }
}

bool NetworkManager::enqueue_tx(const TxJob& tx_job) noexcept
{
    TxJob tmp     = tx_job; // this is awkward
    const UINT st = tx_queue_send(&tx_queue_.queue, &tmp, NX_NO_WAIT);
    return (st == NX_SUCCESS);
}

bool NetworkManager::send_to(
    const int socket_id, const Ip4Endpoint& dst, const std::span<const uint8_t> data
) noexcept
{
    if (data.size() > kMaxUdpPayload)
        return false;

    if (socket_id < 0 || static_cast<size_t>(socket_id) >= binding_count_)
        return false;

    if (!bindings_[socket_id].bound)
        return false;

    NX_PACKET* nx_packet = nullptr;
    UINT st = nx_packet_allocate(&nx_packet_pool_, &nx_packet, NX_UDP_PACKET, NX_NO_WAIT);
    if (st != NX_SUCCESS)
        return false;

    st = nx_packet_data_append(
        nx_packet, const_cast<UCHAR*>(data.data()), data.size(), &nx_packet_pool_, NX_NO_WAIT
    );

    if (st != NX_SUCCESS) {
        nx_packet_release(nx_packet);
        return false;
    }

    TxJob tx_job {};
    tx_job.socket_id = socket_id;
    tx_job.nx_packet = nx_packet;
    tx_job.dst_ip    = dst.address;
    tx_job.dst_port  = dst.port;

    if (!enqueue_tx(tx_job)) {
        nx_packet_release(nx_packet); // Queue full
        return false;
    }

    return true;
}

bool NetworkManager::send_broadcast(
    const int socket_id, const uint16_t port, const std::span<const uint8_t> data
) noexcept
{
    Ip4Endpoint dst {};
    dst.address = kIpv4Broadcast;
    dst.port    = port;
    return send_to(socket_id, dst, data);
}

UINT NetworkManager::receive(
    const int socket_id, RxPacket& packet, const ULONG wait_option
) noexcept
{
    if (socket_id < 0 || static_cast<size_t>(socket_id) >= binding_count_)
        return NX_INVALID_PARAMETERS;

    RxItem rx_item {};
    const UINT q_status = tx_queue_receive(&rx_queues_[socket_id].queue, &rx_item, wait_option);
    if (q_status != NX_SUCCESS) {
        return q_status;
    }

    packet.socket_id = socket_id;

    ULONG src_ip  = 0;
    UINT src_port = 0;
    (void)nx_udp_source_extract(rx_item.nx_packet, &src_ip, &src_port);

    packet.src.address = src_ip;
    packet.src.port    = static_cast<uint16_t>(src_port);

    const UINT cst = copy_payload(rx_item.nx_packet, packet.data, kMaxUdpPayload, packet.len);

    nx_packet_release(rx_item.nx_packet);

    return cst;
}

UINT NetworkManager::copy_payload(
    NX_PACKET* nx_packet, uint8_t* dst, const uint16_t dst_cap, uint16_t& out_len
) noexcept
{
    out_len = 0;

    // Get the payload length
    ULONG total_len = 0;
    UINT status     = nx_packet_length_get(nx_packet, &total_len);
    if (status != NX_SUCCESS)
        return status;

    const ULONG capped = (total_len > dst_cap) ? dst_cap : total_len;

    // Copy the payload
    ULONG b_copied = 0;
    status         = nx_packet_data_extract_offset(nx_packet, 0, dst, capped, &b_copied);
    if (status != NX_SUCCESS)
        return status;

    out_len = static_cast<uint16_t>(b_copied);
    return NX_SUCCESS;
}

} // namespace Network
