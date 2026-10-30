#ifndef CLASS_READY_H
#define CLASS_READY_H

#include "Fsm/States/state_template.h"
#include "Fsm/event_machine.h"
#include "Context.hpp"
#include "Utilities/config.h"

class Ready final : public StateTemplate {
public:
    Ready(std::unordered_map<EVENT, STATE_NAME> transitions)
        : StateTemplate(STATE_NAME::READY, std::move(transitions))
    {
    }

    LOGGER::STATUS enter_new_state(const ULONG& current_time) override;
    LOGGER::STATUS update(const ULONG& current_time) override;

private:
    ULONG m_time_since_thrust = 0;
    ULONG m_missing_heartbeat_ticks = 0;
    bool m_first_missing_heartbeat = true;
};

#endif
