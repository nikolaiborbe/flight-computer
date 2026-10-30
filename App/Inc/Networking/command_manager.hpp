#ifndef CLASS_COMMAND_MANAGER_HPP
#define CLASS_COMMAND_MANAGER_HPP

#include "Networking/commands/GroundStationHeartBeat.hpp"
#include "Networking/commands/command_service.hpp"
#include "Networking/commands/fsm_state_command_service.hpp"
#include "Networking/commands/other_command_service.hpp"
#include "Networking/network_manager.hpp"
#include "commands/test_state_command_service.hpp"
#include <array>
#include <cstddef>

namespace Network {

class CommandManager {
public:
    static constexpr uint16_t kLocalPort   = 25567;
    static constexpr uint8_t kRxQueueDepth = 20;

    explicit CommandManager(NetworkManager& network) noexcept;

    bool start() noexcept;

    void poll() noexcept;

    Services::TestStateCommandService& test_state_service() noexcept
    {
        return test_state_command_service_;
    }

private:
    NetworkManager& network_;
    int socket_id_ = -1;

    Services::FsmStateCommandService fsm_state_command_service_;
    Services::TestStateCommandService test_state_command_service_;
    Services::GroundStationCommandService ground_station_heartbeat_service_;
    Services::OtherCommandService other_command_service_;
#ifdef FC_PYTHON_TESTING
    static constexpr size_t kServiceCount = 3;
#else
    static constexpr size_t kServiceCount = 3;
#endif

#ifdef FC_PYTHON_TESTING
    std::array<Services::CommandService*, kServiceCount> services_ {
        &fsm_state_command_service_, &test_state_command_service_, &ground_station_heartbeat_service_
    };
#else
    std::array<Services::CommandService*, kServiceCount> services_ {
        &fsm_state_command_service_, &ground_station_heartbeat_service_, &other_command_service_
    };
#endif
};

} // namespace Network

#endif // CLASS_COMMAND_MANAGER_HPP
