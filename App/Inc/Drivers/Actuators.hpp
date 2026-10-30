#ifndef NAMESPACE_ACTUATORS_HPP
#define NAMESPACE_ACTUATORS_HPP

// App
#include "Drivers/Servos.hpp"
#include "Drivers/Solenoids.hpp"

// std
#include <memory>

namespace Driver {
class Actuators {
public:
    static std::unique_ptr<Driver::Solenoid> solenoid;
    static std::unique_ptr<Driver::Servo> servo;

    static void init(PCan& pcan);
    static bool set_default_values();
};
} // namespace Driver

#endif