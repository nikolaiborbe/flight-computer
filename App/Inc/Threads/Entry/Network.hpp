#ifndef ENTRY_NETWORK_HPP
#define ENTRY_NETWORK_HPP

// App
#include "Networking/network_context.hpp"
#include "Utilities/config.h"

// std
#include <cstdint>
#include <span>

namespace Thread::Entry {
void Network(unsigned long thread_input);
}

#endif