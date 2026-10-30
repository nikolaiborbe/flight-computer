#ifndef CLASS_STATETEMPLATE_H
#define CLASS_STATETEMPLATE_H

#include "event_machine.h"
#include "logger.h"
#include <stdint.h>
#include <tx_api.h>
#include <memory>
#include <unordered_map>

struct SystemContext;

class StateTemplate {
public:
    StateTemplate(STATE_NAME name, std::unordered_map<EVENT, STATE_NAME> possible_transitions);

    virtual ~StateTemplate() = default;

    /**
     * @brief Enter function that is called once when entering the state.
     * @param current_time The current time.
     */
    virtual LOGGER::STATUS enter_new_state(const ULONG& current_time);

    /**
     * @brief The update function to be called on each FSM iteration.
     * @param current_time The current time.
     */
    virtual LOGGER::STATUS update(const ULONG& current_time);

    /**
     * @brief Function to go to the next state.
     * @details The StateTemplate keeps track in an unordered_map of the transitions <EVENT,
     * STATE_NAME>. This function assumes that the event is in this possible_transitions map, and
     * will use that assumption to get the next state.
     * @warning This is highly risky, but was made this way on purpose. If the event does not exist
     * in the possible_transitions we will get undefined behavior or a crash. We MUST be able to
     * guarantee that the event exist before calling this function. The reason it is made this way
     * is because if this function does cause a crash, that would mean the state machine would have
     * done something we did not expect.
     * @param e The event the next state is associated with.
     */
    STATE_NAME get_next_state(EVENT e);

    /**
     * @brief Get the STATE_NAME for the current state.
     * @returns The name of the current state.
     */
    STATE_NAME get_name();

    /**
     * @brief Set the entry time into the state.
     * @param time The current time.
     * @returns Logger status
     */
    LOGGER::STATUS set_time(const ULONG& time);

    /**
     * @brief Get the entry time into the state.
     * @returns The entry time into the state.
     */
    ULONG get_time() const noexcept;

    const std::unordered_map<EVENT, STATE_NAME>& get_transition_events() noexcept;

protected:
    // if memory usage high change to vector
    std::unordered_map<EVENT, STATE_NAME> m_possible_transitions;

    STATE_NAME m_name;
    ULONG m_entry_time;
};

#endif
