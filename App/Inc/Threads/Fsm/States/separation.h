#ifndef CLASS_SEPARATION_H
#define CLASS_SEPARATION_H

#include "Fsm/States/state_template.h"
#include "Fsm/event_machine.h"
#include "Context.hpp"
#include "Drivers/Actuators.hpp"
#include "Utilities/config.h"

class Separation final : public StateTemplate {
public:
    Separation(std::unordered_map<EVENT, STATE_NAME> transitions)
        : StateTemplate(STATE_NAME::SEPARATION, std::move(transitions))
    {
    }

    LOGGER::STATUS enter_new_state(const ULONG& current_time) override;
    LOGGER::STATUS update(const ULONG& current_time) override;

private:
};

#endif
