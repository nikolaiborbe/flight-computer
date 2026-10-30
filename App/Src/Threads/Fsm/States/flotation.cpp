#include "Fsm/States/flotation.h"
#include "Utilities/config.h"
#include "Context.hpp"

LOGGER::STATUS Flotation::enter_new_state(const ULONG& current_time) 
{
    m_entry_time = current_time;
    Driver::Actuators::solenoid->flotation.open();
    return LOGGER::STATUS::OK;
}

/**
 * TODO:
 * We need to calculate how long it will take for the rocket to 
 * fall all the way down to ~200 m, and use this as a fail safe
 * if Kalman.cpp does not work.
 * 
 * At what speed did Heimdall fall? How long will it take for the
 * rocket to get from ~200 m to 0 m?
 */
LOGGER::STATUS Flotation::update(const ULONG& current_time)
{
    System::kalman_data.refresh();
    const ULONG time_since_flotation = (current_time - m_entry_time);
    const double altitude = System::kalman_data.value().altitude;

    if (altitude <= 200) {
        EventMachine::set_event(EVENT::SHUTDOWN);
    }

    return LOGGER::STATUS::OK;
}
