#include "Fsm/States/coasting.h"
#include "Utilities/config.h"

LOGGER::STATUS Coasting::enter_new_state(const ULONG& current_time)
{
    m_entry_time = current_time;
    m_time_negative_velocity = 0;
    return LOGGER::STATUS::OK;
}

/* TODO: Discuss with people.
 * I believe there is a bug here now where if we reach e.g. Mach 1.1, we would 
 * get a second Mach dip at when it's breaking and therefore in this state, triggering
 * the apogee detection.
 * The solution will probably end up being a timer anyways, but there might be a better
 * solution?
 * 
 * From Heimdall flight: https://www.youtube.com/watch?v=f1tRKNCVl8Y it seems like we
 * rise for ~20 s after burn is completed, I will use a 14 second timer, where this will
 * hopefully catch the Mach dip due to decreasing velocity.
 * 
 * Final note: We are practically certain we wont hit the mach dip, we will keep the 5 s
 * delay, just to be certain, but 14 s is redundant and can fail on low apogee.
*/
LOGGER::STATUS Coasting::update(const ULONG& current_time)
{
    System::kalman_data.refresh();
    System::reduce_network_output_writer.store(false);

    const double altitude = System::kalman_data.value().altitude;
    constexpr double altitude_threashold_m = 150;

    // 5 seconds must pass before we can check for apogee. 
    ULONG time_since_enter = (current_time - m_entry_time);
    if (ms_to_ticks(5000) > time_since_enter)
        return LOGGER::STATUS::OK;


    // TODO: velocity_z (given by kalman, but should be more clear)
    // TODO: Make sure sign on velocity from kalman_data is correct
    const double velocity_m_s = System::kalman_data.value().velocity;

    if (velocity_m_s < 0.0) 
    {
        /**
         * Noisy data could maybe reset this counter?
         */
        if (m_time_negative_velocity == 0)
            m_time_negative_velocity = current_time;

        if ((current_time - m_time_negative_velocity) >= ms_to_ticks(300)) {

            if (altitude > altitude_threashold_m)
                EventMachine::set_event(EVENT::APOGEE_DETECTED);
        }

    } else {
        m_time_negative_velocity = 0;
    }

    return LOGGER::STATUS::OK;
}
