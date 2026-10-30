#include "Context.hpp"
#include "Utilities/utilities_buzzer.h"
#include "stm32h7xx_hal_tim.h"
#include "Drivers/Actuators.hpp"
#include "Threads/Fsm/fsm.h"
#include "Utilities/config.h"
#include "logger.h"

constexpr STATE_NAME FIRST_STATE = STATE_NAME::IDLE;
constexpr EVENT FIRST_EVENT      = EVENT::NONE;

algo::Seqlock<STATE_NAME> StateMachine::m_state_slot;
algo::Seqlock<STATE_NAME>::Writer StateMachine::m_state_writer = StateMachine::m_state_slot.acquire_writer_once();

LOGGER::STATUS StateMachine::init(
    std::unordered_map<STATE_NAME, std::unique_ptr<StateTemplate>> states
)
{
    m_states        = std::move(states);
    m_current_state = m_states.at(FIRST_STATE).get();
    m_current_state->enter_new_state(tx_time_get());
    return LOGGER::STATUS::OK;
}

LOGGER::STATUS StateMachine::setup_devices()
{
    System::current_state.bind(&m_state_slot);
    m_state_writer.store(m_current_state->get_name());
    return LOGGER::STATUS::OK;
}

int i {};
LOGGER::STATUS StateMachine::run()
{
    while (true) {
        // check for transition
        EVENT current_event = EventMachine::get_current_event();

        bool new_event = (current_event != EVENT::NONE);
        if (new_event) {

            STATE_NAME next_state = m_current_state->get_next_state(current_event);
            if (next_state == STATE_NAME::NONE) {
                EventMachine::set_event(EVENT::NONE);
                continue;
            }

            m_current_state = m_states.at(next_state).get();

            // toggle off the event we just reacted to
            EventMachine::set_event(EVENT::NONE);

            m_current_state->enter_new_state(tx_time_get());
        }


        if (i % 1000) {
            play_note(&htim15, TIM_CHANNEL_1, 50, MUSIC_REST, 0);
            i = 0;
        }

        m_current_state->update(tx_time_get());
        m_state_writer.store(m_current_state->get_name());
        tx_thread_sleep(FSM_THREAD_SLEEP_TIMER);
        i++;
    };

    return LOGGER::STATUS::OK;
};
