#include "Networking/commands/command_service.hpp"

namespace Network::Services {

CommandService::CommandService(const char* name, const uint8_t command_id) noexcept
    : name_(name)
    , command_id_(command_id)
{
}

} // namespace Network::Services
