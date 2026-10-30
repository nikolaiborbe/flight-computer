#ifndef ENTRY_PCAN_HPP
#define ENTRY_PCAN_HPP

// App
#include "Context.hpp"
#include "Datastructures/Seqlock.hpp"
#include "Ids/fc_ids.h"

// std

namespace Thread::Entry {
void Pcan(unsigned long thread_input);
}

#endif