#include "Fsm/States/thrusting.h"

LOGGER::STATUS Thrusting::enter_new_state(const ULONG& current_time) 
{
    System::kalman_data.refresh();

    m_entry_time = current_time;
    m_acc_z_under_threshold_since = 0;

    return LOGGER::STATUS::OK;
}

/*
 * TODO: Looking closely at Heimdall footage, a clean way to detect end of thrusting
 * can be just a flipped sign on acceleration in \hat{z}. 
 * 
 * Velocity will be delayed, and will stay at approx. the same velocity for a while,
 * so we might end up predicting end of thrusting with maybe close to 2 s delay.
 * 
*/
LOGGER::STATUS Thrusting::update(const ULONG& current_time)
{
    System::kalman_data.refresh();
    System::data.icm_data.refresh();

    bool finished_thrusting = false;
    bool acceleration_under_threshold = false;
    constexpr float acceleration_threshold_z_m_per_ss = 3*G;

    // TODO: check sign and drop abs
    const float acc_z_m_per_ss = std::abs(System::data.icm_data.value().acc.acc_z_m_per_ss);
    const float velocity_z_m_s = std::abs(System::kalman_data.value().velocity);

    // TODO: make sure acceleration sign is correct.
    if (acc_z_m_per_ss < acceleration_threshold_z_m_per_ss)
    {
        if (m_acc_z_under_threshold_since == 0)
            m_acc_z_under_threshold_since = current_time;

        if ((current_time - m_acc_z_under_threshold_since) >= ms_to_ticks(100))
            acceleration_under_threshold = true;
    } else 
    {
        m_acc_z_under_threshold_since = 0;
    }


    /**
     * Check if the rocket gained speed and stopped accelerating.
     * Simply checking if acceleration stops could trigger a recovery
     * sequence if the rocket is stuck on the pad.
    */
    finished_thrusting = acceleration_under_threshold;
    if (finished_thrusting && (velocity_z_m_s > 20))
    {
        EventMachine::set_event(EVENT::THRUSTING_FINISHED);
    } 

    return LOGGER::STATUS::OK;
}
