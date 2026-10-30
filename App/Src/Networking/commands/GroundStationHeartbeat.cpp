#include "Networking/commands/GroundStationHeartBeat.hpp"

#include "Context.hpp"
#include "Utilities/config.h"

namespace Network::Services {

namespace {
    constexpr uint8_t  kCommandId       = 0x11;
    // Ground station beats every 15 s; allow one late beat before going stale.
    constexpr uint32_t kBeatIntervalMs  = 10'000;
    constexpr uint32_t kBeatTimeoutMs   = kBeatIntervalMs + 3'000;
}

GroundStationCommandService::GroundStationCommandService() noexcept
    : CommandService("GroundStationHeartBeat", kCommandId)
{
}

void GroundStationCommandService::handle(std::span<const uint8_t> payload) noexcept
{
    (void)payload; // an empty beat is all we need
    last_beat_ticks_ = static_cast<uint32_t>(tx_time_get());
    beat_seen_       = true;
}

void GroundStationCommandService::tick(uint32_t now_ticks) noexcept
{
    const uint32_t elapsed = now_ticks - last_beat_ticks_; // unsigned: wrap-safe
    const bool alive       = beat_seen_ && elapsed <= ms_to_ticks(kBeatTimeoutMs);

    if (alive != alive_) {
        alive_ = alive;
        System::ground_station_heartbeat_writer.store(alive);
    }
}

} // namespace Network::Services