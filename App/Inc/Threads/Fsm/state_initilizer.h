/**
 * @file
 * @author Mikhail mikhail.rohloff@propulsentnu.no
 * @author Nikolai G. Borbe nikolai.borbe@propulsentnu.no
 * @brief initialization of the states for the FSM.
 * @version 0.1
 * @date 2025-10-25
 *
 * @copyright Copyright Propulse NTNU (c) 2024
 */

#ifndef STATES_H
#define STATES_H

#include "Fsm/States/state_template.h"
#include "Fsm/event_machine.h"

// states
#include "Fsm/States/idle.h"
#include "Fsm/States/ready.h"
#include "Fsm/States/thrusting.h"
#include "Fsm/States/coasting.h"
#include "Fsm/States/separation.h"
#include "Fsm/States/flotation.h"
#include "Fsm/States/filling.h"
#include "Fsm/States/flotation_filling.h"
#include "Fsm/States/separation_filling.h"
#include "Fsm/States/shutdown.h"


extern std::unordered_map<STATE_NAME, std::unique_ptr<StateTemplate>> states;

/**
 * @brief Initilizer for the states.
 */
void init_states();

#endif
