#include "Fsm/States/shutdown.h"
#include "States/state_template.h"
#include "fc_ids.h"
#include "Context.hpp"

/**
 * TODO:
 * This needs to be implemented and tested, we will shut off power
 * when we enter this state.
 * 
 * This needs to be tested once more with the new code, a simple step through 
 * should be fine. 
 */
LOGGER::STATUS Shutdown::enter_new_state(const ULONG& current_time)
{
    m_entry_time = current_time;
    return LOGGER::STATUS::OK;
}

LOGGER::STATUS Shutdown::update(const ULONG& current_time)
{
    uint8_t data = 69;
    while (true) {
        System::pcan1->publish_command(PCAN_PRIORITY_LOW, FC_BMS_POINT_ENDPOINT, &data, sizeof(data));
        tx_thread_sleep(10);
    }
    return LOGGER::STATUS::OK;
}
