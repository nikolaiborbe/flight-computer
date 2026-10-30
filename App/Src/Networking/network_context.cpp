#include "Networking/network_context.hpp"

namespace Network
{
    std::shared_ptr<NetworkManager> manager = nullptr;
    std::shared_ptr<BroadcastManager> broadcaster = nullptr;
    std::shared_ptr<CommandManager> commands = nullptr;
}
