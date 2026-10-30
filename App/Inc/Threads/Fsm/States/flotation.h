#ifndef CLASS_FLOTATION_H
#define CLASS_FLOTATION_H

#include "Fsm/States/state_template.h"
#include "Fsm/event_machine.h"
#include "Drivers/Actuators.hpp"

class Flotation final : public StateTemplate {
public:
    Flotation(std::unordered_map<EVENT, STATE_NAME> transitions)
        : StateTemplate(STATE_NAME::FLOTATION, std::move(transitions))
    {
    }

    LOGGER::STATUS enter_new_state(const ULONG& current_time) override;
    LOGGER::STATUS update(const ULONG& current_time) override;

private:
};

#endif