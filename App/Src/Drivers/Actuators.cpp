#include "Drivers/Actuators.hpp"

namespace Driver {
std::unique_ptr<Solenoid> Actuators::solenoid = nullptr;
std::unique_ptr<Servo> Actuators::servo       = nullptr;

void Actuators::init(PCan& pcan)
{
    solenoid = std::make_unique<Solenoid>(pcan);
    servo    = std::make_unique<Servo>(pcan);
}

bool Actuators::set_default_values()
{
    solenoid->flotation.close();
    solenoid->flotation_filling.close();
    solenoid->separation_filling.close();
    servo->separation.servo.beginPosition({0});
    return true;
}
} // namespace Driver