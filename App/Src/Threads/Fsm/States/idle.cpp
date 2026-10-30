#include "Fsm/States/idle.h"
#include "Context.hpp"
#include "Utilities/config.h"
#include "event_machine.h"

LOGGER::STATUS Idle::enter_new_state(const ULONG& current_time)
{
    u8 msg = 0;
    System::pcan2->publish_command(PCAN_PRIORITY_LOW, {6, 0}, &msg, 1); // Turn off cameras

    m_missing_heartbeat_ticks = 0;

    m_entry_time = current_time;
    Driver::Actuators::set_default_values();
    return LOGGER::STATUS::OK;
}

LOGGER::STATUS Idle::update(const ULONG& current_time)
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