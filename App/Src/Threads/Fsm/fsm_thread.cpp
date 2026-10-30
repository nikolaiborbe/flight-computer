#include "Threads/Fsm/fsm.h"
#include "Threads/Fsm/state_initilizer.h"
#include "Utilities/config.h"

extern "C" void FSM_thread_entry(ULONG thread_input)
{
    (void)thread_input;
    
    init_states();

    StateMachine fsm;
    fsm.init(std::move(states));

    while (fsm.setup_devices() != LOGGER::STATUS::OK)
    {
        tx_thread_sleep(ms_to_ticks(1000));
    }

    fsm.run();
}
