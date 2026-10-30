#include "Fsm/States/separation.h"


LOGGER::STATUS Separation::enter_new_state(const ULONG& current_time)
{
    m_entry_time = current_time;

    Driver::Actuators::servo->separation.servo.beginPosition({90});
    return LOGGER::STATUS::OK;
}

LOGGER::STATUS Separation::update(const ULONG& current_time)
{
    const ULONG time_since_separation = (current_time - m_entry_time);
    const bool valid_time = (time_since_separation >= ms_to_ticks(3000));

    if (valid_time) 
    {
        EventMachine::set_event(EVENT::FLOTATION);
    };

    return LOGGER::STATUS::OK;
}
