#include "Networking/commands/other_command_service.hpp"
#include "command_objects.hpp"
#include "Utilities/utilities_buzzer.h"
#include "event_machine.h"
#include "stm32h723xx.h"
#include "Context.hpp"
#include "tx_api.h"
#include <cstring>

namespace Network::Services {

namespace {
    constexpr uint8_t kCommandId = 0x12;
    constexpr ULONG kBadAppleStackSize = 2 * 1024;
    constexpr UINT kBadApplePriority = 20;

    u8 last_camera_state = 0;
    TX_THREAD bad_apple_thread {};
    alignas(8) uint8_t bad_apple_stack[kBadAppleStackSize] {};
    bool bad_apple_thread_created = false;

    void bad_apple_thread_entry(ULONG)
    {
        play_bad_apple();
    }

    void start_bad_apple_thread()
    {
        if (bad_apple_thread_created) {
            const UINT state = bad_apple_thread.tx_thread_state;

            if (state == TX_COMPLETED || state == TX_TERMINATED) {
                if (tx_thread_delete(&bad_apple_thread) != TX_SUCCESS) {
                    return;
                }

                bad_apple_thread_created = false;
            } else {
                return;
            }
        }

        const UINT status = tx_thread_create(
            &bad_apple_thread,
            const_cast<char*>("Bad Apple"),
            bad_apple_thread_entry,
            0,
            bad_apple_stack,
            sizeof(bad_apple_stack),
            kBadApplePriority,
            kBadApplePriority,
            TX_NO_TIME_SLICE,
            TX_AUTO_START
        );

        bad_apple_thread_created = status == TX_SUCCESS;
    }
}

OtherCommandService::OtherCommandService() noexcept
    : CommandService("OtherCommands", kCommandId)
{
}

void OtherCommandService::handle(std::span<const uint8_t> payload) noexcept
{
    if (payload.size() < sizeof(Commands::OtherCommandsGUI))
        return;

    Commands::OtherCommandsGUI command {};
    std::memcpy(&command, payload.data(), sizeof(command));

    switch (command.cmd) {
    case Commands::OtherCommand::TOGGLE_CAMERA:
        last_camera_state = last_camera_state ? 0 : 1;
        System::pcan2->publish_command(PCAN_PRIORITY_LOW, {6, 0}, &last_camera_state, 1); // Turn on cameras
        break;
    case Commands::OtherCommand::FORMAT_SD:
        System::storage_manager->request_format(); // format SD card
        break;
    }
}

} // namespace Network::Services
