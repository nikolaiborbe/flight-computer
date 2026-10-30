#ifndef ENTRY_FSM_HPP
#define ENTRY_FSM_HPP

// App
#include "Fsm/fsm.h"
#include "Fsm/state_initilizer.h"

// std
#include <utility>

namespace Thread::Entry {
void FSM(ULONG thread_input);
} // namespace Thread::Entry

#endif