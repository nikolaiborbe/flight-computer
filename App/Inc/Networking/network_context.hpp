#pragma once

#include "Networking/broadcast_manager.hpp"
#include "Networking/command_manager.hpp"
#include "Networking/network_manager.hpp"
#include <memory>

namespace Network {
extern std::shared_ptr<NetworkManager> manager;
extern std::shared_ptr<BroadcastManager> broadcaster;
extern std::shared_ptr<CommandManager> commands;
} // namespace Network
