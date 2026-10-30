#include "Threads/Entry/FSM.hpp"
#include "Data/Writers.hpp"

namespace Thread
{
    namespace Entry
    {
        void FSM(ULONG thread_input)
        {
            auto writer = reinterpret_cast<Telemetry::Writers::Fsm*>(thread_input);
            writer->fsm_thread_online.store(true);

            init_states();

            StateMachine FSM;
            FSM.init(std::move(states));
            LOGGER::STATUS status = LOGGER::STATUS::INITIALIZE_ERROR;
            while (status != LOGGER::STATUS::OK)
            {
                status = FSM.setup_devices();
                tx_thread_sleep(RTOS_MS_TO_TICKS(1000));
            }

            FSM.run();
            writer->fsm_thread_online.store(false);
        }
    }
}