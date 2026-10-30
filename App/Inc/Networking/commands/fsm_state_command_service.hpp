#ifndef CLASS_FSM_STATE_COMMAND_SERVICE_HPP
#define CLASS_FSM_STATE_COMMAND_SERVICE_HPP

#include "Networking/command_objects.hpp"
#include "Networking/commands/command_service.hpp"

namespace Network::Services {

// Handles Commands::FSMStateCommand (command_objects.hpp "#id=0x10").
class FsmStateCommandService : public CommandService {
public:
    FsmStateCommandService() noexcept;

    void handle(std::span<const uint8_t> payload) noexcept override;
};

} // namespace Network::Services

#endif // CLASS_FSM_STATE_COMMAND_SERVICE_HPP
