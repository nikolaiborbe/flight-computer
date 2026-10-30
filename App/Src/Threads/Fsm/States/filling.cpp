#include "Fsm/States/filling.h"

LOGGER::STATUS Filling::enter_new_state(const ULONG& current_time)
{
    m_entry_time = current_time;
    Driver::Actuators::solenoid->flotation_filling.open();
    Driver::Actuators::solenoid->separation_filling.open();

    return LOGGER::STATUS::OK;
}

LOGGER::STATUS Filling::update(const ULONG& current_time)
{
    return LOGGER::STATUS::OK;
}
