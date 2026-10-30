#include "Networking/commands/fsm_state_command_service.hpp"
#include "command_objects.hpp"
#include "event_machine.h"
#include "stm32h723xx.h"
#include "Context.hpp"
#include <cstring>

namespace Network::Services {

namespace {
    constexpr uint8_t kCommandId = 0x10;
}

FsmStateCommandService::FsmStateCommandService() noexcept
    : CommandService("FsmState", kCommandId)
{
}

void FsmStateCommandService::handle(std::span<const uint8_t> payload) noexcept
{
    if (payload.size() < sizeof(Commands::FSMStateCommand))
        return;

    Commands::FSMStateCommand command {};
    std::memcpy(&command, payload.data(), sizeof(command));

    switch (command.command) {
    case Commands::StateCommands::IDLE:
        EventMachine::set_event(EVENT::IDLE);
        break;
    case Commands::StateCommands::OPEN_FILLING:
        EventMachine::set_event(EVENT::OPEN_FILL_CMD);
        break;
    case Commands::StateCommands::OPEN_SEPARATION_FILL:
        EventMachine::set_event(EVENT::OPEN_SEPARATION_FILL_CMD);
        break;
    case Commands::StateCommands::EXPECT_LIFTOFF:
        EventMachine::set_event(EVENT::EXPECT_LIFTOFF);
        break;
    case Commands::StateCommands::OPEN_FLOTATION_FILL:
        EventMachine::set_event(EVENT::OPEN_FLOTATION_FILL_CMD);
        break;
    case Commands::StateCommands::OPEN_SEPARATION:
        EventMachine::set_event(EVENT::OPEN_SEPARATION);
        break;
    }
}

} // namespace Network::Services
