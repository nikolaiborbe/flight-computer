#ifndef CLASS_FLOTATION_FILLING_H
#define CLASS_FLOTATION_FILLING_H

#include "Fsm/States/state_template.h"
#include "Fsm/event_machine.h"
#include "Drivers/Actuators.hpp"

class FlotationFilling final : public StateTemplate {
public:
    FlotationFilling(std::unordered_map<EVENT, STATE_NAME> transitions)
        : StateTemplate(STATE_NAME::FLOTATION_FILLING, std::move(transitions))
    {
    }

    LOGGER::STATUS enter_new_state(const ULONG& current_time) override;
    LOGGER::STATUS update(const ULONG& current_time) override;
private:
    ULONG m_missing_heartbeat_ticks = 0;
    bool m_first_missing_heartbeat = true;
};

#endif
