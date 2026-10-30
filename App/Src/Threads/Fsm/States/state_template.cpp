#include "Fsm/States/state_template.h"
#include "event_machine.h"
#include "logger.h"
#include <stdint.h>

StateTemplate::StateTemplate(
    STATE_NAME name, std::unordered_map<EVENT, STATE_NAME> possible_transitions
)
    : m_entry_time(0)
    , m_name(name)
    , m_possible_transitions(possible_transitions)
{
}

LOGGER::STATUS StateTemplate::enter_new_state(const ULONG& current_time)
{
    this->m_entry_time = current_time;

    return LOGGER::STATUS::OK;
}

LOGGER::STATUS StateTemplate::update(const ULONG& current_time)
{
    return LOGGER::STATUS::OK;
}

STATE_NAME StateTemplate::get_next_state(EVENT e)
{
    if (this->m_possible_transitions.count(e) == 0) {
        return STATE_NAME::NONE;
    }

    return this->m_possible_transitions[e];
}

LOGGER::STATUS StateTemplate::set_time(const ULONG& time)
{
    m_entry_time = time;
    return LOGGER::STATUS::OK;
}

ULONG StateTemplate::get_time() const noexcept { return m_entry_time; }

STATE_NAME StateTemplate::get_name() { return m_name; }

const std::unordered_map<EVENT, STATE_NAME>& StateTemplate::get_transition_events() noexcept
{
    return m_possible_transitions;
}
