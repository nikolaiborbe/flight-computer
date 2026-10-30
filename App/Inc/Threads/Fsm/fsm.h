/**
 * @file
 * @author Mikhail mikhail.rohloff@propulsentnu.no
 * @author Nikolai G. Borbe nikolai.borbe@propulsentnu.no
 * @brief Implementation of the finite state machine (FSM) for the rocket.
 * @version 0.1
 * @date 2025-10-25
 *
 * @copyright Copyright Propulse NTNU (c) 2024
 */

#ifndef CLASS_FSM_H
#define CLASS_FSM_H

// stm
#include "Datastructures/Seqlock.hpp"
#include "main.h"

// vendor
#include "can_solenoid.hpp"
#include "pcan_core.h"
#include "logger.h"
#include "fc_ids.h"

// App
#include "States/state_template.h"
#include "Threads/Fsm/fsm_pressure.h"
#include "Threads/Fsm/event_machine.h"
#include "Context.hpp"
#include "Data/Local.hpp"

// std
#include <array>
#include <memory>
#include <vector>


class StateMachine {
private:
    StateTemplate* m_current_state;
    std::unordered_map<STATE_NAME, std::unique_ptr<StateTemplate>> m_states;

    static algo::Seqlock<STATE_NAME> m_state_slot;
    static algo::Seqlock<STATE_NAME>::Writer m_state_writer;
public:
    StateMachine()  = default;
    ~StateMachine() = default;

    /**
     * @brief Initilizing the FSM.
     * @param states The map with all the states.
     * @returns The logger status.
     */
    LOGGER::STATUS init(
        std::unordered_map<STATE_NAME, std::unique_ptr<StateTemplate>> states
    );

    /**
     * @brief Main loop for the FSM.
     * @returns The logger status.
     */
    LOGGER::STATUS run();
    
    /**
     * @brief Setup and initilize sensors and actuator interfaces.
     * @returns The logger status.
     */
    LOGGER::STATUS setup_devices();
};

extern "C" void FSM_thread_entry(ULONG thread_input);

#endif
