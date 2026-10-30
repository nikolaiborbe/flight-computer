#ifndef GROUND_STATION_HEART_BEAT_HPP
#define GROUND_STATION_HEART_BEAT_HPP

#include "command_objects.hpp"
#include "command_service.hpp"

namespace Network::Services {

// Watches the ground station heartbeat (command id 0x11). Every inbound beat
// refreshes a timestamp; tick() publishes System::ground_station_heartbeat,
// which stays true while beats keep arriving and flips false once one is missed.
class GroundStationCommandService : public CommandService {
public:
    GroundStationCommandService() noexcept;

    void handle(std::span<const uint8_t> payload) noexcept override;
    void tick(uint32_t now_ticks) noexcept override;

private:
    uint32_t last_beat_ticks_ = 0;
    bool     beat_seen_       = false;
    bool     alive_           = false;
};
}

#endif