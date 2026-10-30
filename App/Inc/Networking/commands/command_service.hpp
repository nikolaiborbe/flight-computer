#ifndef CLASS_COMMAND_SERVICE_HPP
#define CLASS_COMMAND_SERVICE_HPP

#include <cstdint>
#include <span>

namespace Network::Services {

// One inbound command type: a command id (command_objects.hpp "#id=") and a
// handler for whatever payload follows it. CommandManager owns the shared
// control-port socket and routes each packet's leading id byte to whichever
// registered CommandService matches.
class CommandService {
public:
    CommandService(const char* name, uint8_t command_id) noexcept;
    virtual ~CommandService() = default;

    CommandService(const CommandService&)            = delete;
    CommandService& operator=(const CommandService&) = delete;

    // payload is everything in the packet after the id byte.
    virtual void handle(std::span<const uint8_t> payload) noexcept = 0;

    // Called once per CommandManager poll with the current RTOS tick, so a
    // service can run time-based logic (e.g. watchdogs) even with no traffic.
    virtual void tick(uint32_t now_ticks) noexcept { (void)now_ticks; }

    [[nodiscard]] const char* name() const noexcept { return name_; }
    [[nodiscard]] uint8_t command_id() const noexcept { return command_id_; }

private:
    const char* name_;
    uint8_t command_id_;
};

} // namespace Network::Services

#endif // CLASS_COMMAND_SERVICE_HPP
