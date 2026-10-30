#include "event_machine.h"
TX_MUTEX EventMachine::event_mutex;

void EventMachine::init()
{
    tx_mutex_create(&EventMachine::event_mutex, (CHAR*)"event_mutex", TX_INHERIT);
}

EVENT EventMachine::m_current_event = EVENT::NONE;

EVENT EventMachine::get_current_event() noexcept
{
    tx_mutex_get(&EventMachine::event_mutex, TX_WAIT_FOREVER);
    EVENT e =  m_current_event;
    tx_mutex_put(&EventMachine::event_mutex);
    return e;
}

bool EventMachine::is_event_active(EVENT event)
{
    tx_mutex_get(&EventMachine::event_mutex, TX_WAIT_FOREVER);
    bool b = m_current_event == event;
    tx_mutex_put(&EventMachine::event_mutex);
    return b;
}

// true = on, false = off
LOGGER::STATUS EventMachine::set_event(EVENT event)
{
    tx_mutex_get(&EventMachine::event_mutex, TX_WAIT_FOREVER);
    m_current_event = event;
    tx_mutex_put(&EventMachine::event_mutex);
    return LOGGER::STATUS::OK;
}
