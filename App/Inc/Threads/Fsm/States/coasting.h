#ifndef CLASS_COASTING_H
#define CLASS_COASTING_H

#include "Fsm/States/state_template.h"
#include "Fsm/event_machine.h"
#include "Context.hpp"

class Coasting final : public StateTemplate {
public:
    Coasting(std::unordered_map<EVENT, STATE_NAME> transitions)
        : StateTemplate(STATE_NAME::COASTING, std::move(transitions))
    {
    }

    LOGGER::STATUS enter_new_state(const ULONG& current_time) override;
    LOGGER::STATUS update(const ULONG& current_time) override;

private:
    ULONG m_time_negative_velocity = 0;
};

#endif
