#ifndef CLASS_IDLE_H
#define CLASS_IDLE_H

#include "Fsm/States/state_template.h"
#include "Fsm/event_machine.h"
#include "main.h"
#include "stm32h7xx_hal_gpio.h"
#include "Drivers/Actuators.hpp"

class Idle final : public StateTemplate {
public:
    Idle(std::unordered_map<EVENT, STATE_NAME> transitions)
        : StateTemplate(STATE_NAME::IDLE, std::move(transitions))
    {
    }

    LOGGER::STATUS enter_new_state(const ULONG& current_time) override;
    LOGGER::STATUS update(const ULONG& current_time) override;

private:
    ULONG m_missing_heartbeat_ticks = 0;
    bool m_first_missing_heartbeat = true;
};

#endif
