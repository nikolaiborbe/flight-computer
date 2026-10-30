#ifndef CLASS_OPEN_SEPARATION_SERVO_HPP
#define CLASS_OPEN_SEPARATION_SERVO_HPP

#include "Fsm/States/state_template.h"
#include "Fsm/event_machine.h"
#include "main.h"
#include "stm32h7xx_hal_gpio.h"
#include "Drivers/Actuators.hpp"

class open_separation_servo final : public StateTemplate {
public:
    open_separation_servo(std::unordered_map<EVENT, STATE_NAME> transitions)
        : StateTemplate(STATE_NAME::OPEN_SEPARATION_SERVO, std::move(transitions))
    {
    }

    LOGGER::STATUS enter_new_state(const ULONG& current_time) override;
    LOGGER::STATUS update(const ULONG& current_time) override;

private:
    ULONG m_missing_heartbeat_ticks = 0;
};

#endif
