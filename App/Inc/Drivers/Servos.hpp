#ifndef NAMESPACE_SERVOS_HPP
#define NAMESPACE_SERVOS_HPP

// vendor
#include "Ids/fc_ids.h"
#include "can_servo.hpp"

namespace Driver {

struct Servo {
    struct ServoType {
        actuators::servo::CanServo servo;
        actuators::servo::PositionCommand position_command {};
        actuators::servo::CommandResult command_result
            = actuators::servo::CommandResult::RejectedNotReady;

        ServoType(PCan& pcan, pcan_endpoint_id_t id)
            : servo(pcan, id)
        {
        }
    };

    ServoType separation;

    explicit Servo(PCan& pcan)
        : separation(pcan, pcan_endpoint_id_t {FC_SERVO_SEPARATION})
    {
    }
};

} // namespace Driver

#endif