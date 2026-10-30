#ifndef CLASS_SHUTDOWN_H
#define CLASS_SHUTDOWN_H

#include "Fsm/States/state_template.h"
#include "Fsm/event_machine.h"
#include "Context.hpp"
#include "Utilities/config.h"

class Shutdown final : public StateTemplate {
public:
    Shutdown(std::unordered_map<EVENT, STATE_NAME> transitions)
        : StateTemplate(STATE_NAME::SHUTDOWN, std::move(transitions))
    {
    }

    LOGGER::STATUS enter_new_state(const ULONG& current_time) override;
    LOGGER::STATUS update(const ULONG& current_time) override;

private:
};

#endif
