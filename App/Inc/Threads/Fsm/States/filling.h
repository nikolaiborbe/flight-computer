#ifndef CLASS_FILLING_H
#define CLASS_FILLING_H

#include "Fsm/States/state_template.h"
#include "Fsm/event_machine.h"
#include "Drivers/Actuators.hpp"

class Filling final : public StateTemplate {
public:
    Filling(std::unordered_map<EVENT, STATE_NAME> transitions)
        : StateTemplate(STATE_NAME::FILLING, std::move(transitions))
    {
    }

    LOGGER::STATUS enter_new_state(const ULONG& current_time) override;
    LOGGER::STATUS update(const ULONG& current_time) override;

private:
};

#endif
