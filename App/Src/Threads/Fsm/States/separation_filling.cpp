#include "Fsm/States/separation_filling.h"
#include "Context.hpp"
#include "Utilities/config.h"

LOGGER::STATUS SeparationFilling::enter_new_state(const ULONG& current_time)
{
    m_entry_time = current_time;

    m_missing_heartbeat_ticks = 0;

    Driver::Actuators::solenoid->separation_filling.open();

    return LOGGER::STATUS::OK;
}

LOGGER::STATUS SeparationFilling::update(const ULONG& current_time)
{
    if (!System::ground_station_heartbeat.update_and_read()) {
        if (m_missing_heartbeat_ticks == 0) {
            m_missing_heartbeat_ticks = current_time;
        }

        if ((current_time - m_missing_heartbeat_ticks) >= ms_to_ticks(2 * 60 * 1000)) {
            EventMachine::set_event(EVENT::LOST_GROUNDSTATION_CONNECTION);
        }
    } else {
        m_missing_heartbeat_ticks = 0;
    }

    return LOGGER::STATUS::OK;
}