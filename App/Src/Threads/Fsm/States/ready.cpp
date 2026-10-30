#include "Fsm/States/ready.h"
#include "event_machine.h"
#include "Utilities/config.h"
#include "Context.hpp"
#include "fc_ids.h"

LOGGER::STATUS Ready::enter_new_state(const ULONG& current_time)
{
    u8 msg = 1;
    System::pcan2->publish_command(PCAN_PRIORITY_LOW, {6, 0}, &msg, 1); // Turn on cameras
    m_missing_heartbeat_ticks = 0;

    m_entry_time = current_time;
    m_time_since_thrust = 0;
    return LOGGER::STATUS::OK;
}

LOGGER::STATUS Ready::update(const ULONG& current_time)
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

    // TODO: Make sure this is oriented correctly.
    System::data.icm_data.refresh();
    float acc_z = std::abs(System::data.icm_data.value().acc.acc_z_m_per_ss);

    /* Loadcells from hotfire testing makes it look like a slightly gradual
     * increase in acceleration, leaving it at 3G would mean we could 
     * detect takeoff slightly late. Since we will only go into the ready state,
     * when the rocket is already on the rail and ready for launch, 2G is a good
     * compromise. 
     * 
     * We are keeping 3G, this is to decrease the chance of a false takeoff detection.
    */
    if (acc_z > 3*G) {
        if (m_time_since_thrust == 0) {
            m_time_since_thrust = current_time;
        }
        if ((current_time - m_time_since_thrust) >= ms_to_ticks(100)) {
            EventMachine::set_event(EVENT::THRUSTING_DETECTED);
        }
    } else {
        m_time_since_thrust = 0;
    }

    return LOGGER::STATUS::OK;
}
