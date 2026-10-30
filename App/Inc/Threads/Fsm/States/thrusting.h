#ifndef CLASS_THRUSTING_H
#define CLASS_THRUSTING_H

#include "Fsm/States/state_template.h"
#include "Fsm/event_machine.h"
#include "Context.hpp"
#include "Utilities/config.h"

class Thrusting final : public StateTemplate {
public:
    Thrusting(std::unordered_map<EVENT, STATE_NAME> transitions)
        : StateTemplate(STATE_NAME::THRUSTING, std::move(transitions))
    {
    }

    LOGGER::STATUS enter_new_state(const ULONG& current_time) override;
    LOGGER::STATUS update(const ULONG& current_time) override;

private:
    ULONG m_acc_z_under_threshold_since = 0;
};

#endif
