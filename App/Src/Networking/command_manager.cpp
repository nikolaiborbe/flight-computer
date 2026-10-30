#include "Networking/command_manager.hpp"

#include "tx_api.h"

namespace Network {

CommandManager::CommandManager(NetworkManager& network) noexcept
    : network_(network)
{
}

bool CommandManager::start() noexcept
{
    socket_id_ = network_.bind("Command", kLocalPort, kRxQueueDepth);
    return socket_id_ >= 0;
}

void CommandManager::poll() noexcept
{
    if (socket_id_ < 0)
        return;

    RxPacket packet {};
    while (network_.receive(socket_id_, packet, NX_NO_WAIT) == NX_SUCCESS) {
        if (packet.len == 0)
            continue;

        const uint8_t command_id = packet.data[0];
        const std::span<const uint8_t> payload(packet.data + 1, packet.len - 1);

        for (Services::CommandService* service : services_) {
            if (service->command_id() == command_id) {
                service->handle(payload);
                break;
            }
        }
    }

    // Drive time-based service logic (heartbeat watchdog) every poll, so the
    // heartbeat can go stale even when no packets are arriving.
    const uint32_t now_ticks = static_cast<uint32_t>(tx_time_get());
    for (Services::CommandService* service : services_) {
        service->tick(now_ticks);
    }
}

} // namespace Network
