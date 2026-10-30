#ifndef CLASS_TEST_STATE_COMMAND_SERVICE_HPP
#define CLASS_TEST_STATE_COMMAND_SERVICE_HPP

#include "Data/Writers.hpp"
#include "commands/command_service.hpp"

/*
 * Already deprecated lol
 * Needs more work and will probably be expanded on in the future.
 * Until launch let it stay like this
 */
namespace Network::Services
{
    class TestStateCommandService : public CommandService
    {
    public:
        TestStateCommandService() noexcept;
        void handle(std::span<const uint8_t> payload) noexcept override;

    private:
    };
}

#endif
