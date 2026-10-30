/**
 * @file
 * @author Mikhail mikhail.rohloff@propulsentnu.no
 * @author Nikolai G. Borbe nikolai.borbe@propulsentnu.no
 * @brief Implementation of an event machine used to keep track of events.
 * @version 0.1
 * @date 2025-10-25
 *
 * @copyright Copyright Propulse NTNU (c) 2025
 */

#ifndef CLASS_EVENTMACHINE_H
#define CLASS_EVENTMACHINE_H

#include "logger.h"
#include "Data/Local.hpp"

enum EVENT {
    NONE,
    IDLE,
    LOST_GROUNDSTATION_CONNECTION,

    /* Commands */
    OPEN_FILL_CMD,
    OPEN_FLOTATION_FILL_CMD,
    OPEN_SEPARATION_FILL_CMD,
    EXPECT_LIFTOFF,

    /* EMERGENCY */
    OPEN_SEPARATION,

    LIFTOFF,

    THRUSTING_DETECTED,
    THRUSTING_FINISHED,

    BURN_DETECTED,
    APOGEE_DETECTED,
    FLOTATION,
    SHUTDOWN,
};


/* 
 * These are vibecoded and used for generating a diagram over state transitions
 * It's kind of sketch, but a lot better than to not check visually.
*/
/* inline const char* to_string(EVENT e)
{
    switch (e) {
    case NONE: return "NONE";
    case BOOT: return "BOOT";
    case OPEN_FILL_CMD: return "OPEN_FILL_CMD";
    case OPEN_FLOTATION_FILL_CMD: return "OPEN_FLOTATION_FILL_CMD";
    case OPEN_SEPARATION_FILL_CMD: return "OPEN_SEPARATION_FILL_CMD";
    case EXPECT_LIFTOFF: return "EXPECT_LIFTOFF";
    case IDLE: return "IDLE";
    case THRUSTING_DETECTED: return "THRUSTING_DETECTED";
    case THRUSTING_FINISHED: return "THRUSTING_FINISHED";
    case APOGEE_DETECTED: return "APOGEE_DETECTED";
    case FLOTATION: return "FLOTATION";
    case SHUTDOWN: return "SHUTDOWN";
    }
    return "UNKNOWN_EVENT";
}

inline const char* to_string(STATE_NAME s)
{
    switch (s) {
    case STATE_NAME::NONE: return "NONE";
    case STATE_NAME::IDLE: return "IDLE";
    case STATE_NAME::FILLING: return "FILLING";
    case STATE_NAME::SEPARATION_FILLING: return "SEPARATION_FILLING";
    case STATE_NAME::FLOTATION_FILLING: return "FLOTATION_FILLING";
    case STATE_NAME::READY: return "READY";
    case STATE_NAME::FLOTATION: return "FLOTATION";
    case STATE_NAME::SEPARATION: return "SEPARATION";
    case STATE_NAME::COASTING: return "COASTING";
    case STATE_NAME::THRUSTING: return "THRUSTING";
    case STATE_NAME::SHUTDOWN: return "SHUTDOWN";
    }
    return "UNKNOWN_STATE";
} */

class EventMachine {
private:
    static TX_MUTEX event_mutex;

public:
    static void init();

    /**
     * @brief Check if event is active.
     * @param event The current active event.
     * @returns boolean value for the state of the event.
     */
    static bool is_event_active(EVENT event);

    /**
     * @brief The the current event.
     * @param event The current active event.
     */
    static LOGGER::STATUS set_event(EVENT event);

    /**
     * @brief Get the current event.
     * @returns The active event.
     */
    static EVENT get_current_event() noexcept;

private:
    static EVENT m_current_event;
};

#endif
