#ifndef CLASS_OTHER_COMMAND_SERVICE_HPP
#define CLASS_OTHER_COMMAND_SERVICE_HPP

#include "Networking/command_objects.hpp"
#include "Networking/commands/command_service.hpp"

namespace Network::Services {

class OtherCommandService : public CommandService {
public:
    OtherCommandService() noexcept;

    void handle(std::span<const uint8_t> payload) noexcept override;
};

} // namespace Network::Services

#endif // CLASS_OTHER_COMMAND_SERVICE_HPP
