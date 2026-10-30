#include "Threads/Fsm/States/open_separation_servo.hpp"


LOGGER::STATUS open_separation_servo::enter_new_state(const ULONG& current_time)
{
    Driver::Actuators::servo->separation.servo.beginPosition({90});
    return LOGGER::OK;
}

LOGGER::STATUS open_separation_servo::update(const ULONG& current_time)
{
    return LOGGER::OK;
}