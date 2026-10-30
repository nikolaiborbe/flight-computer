#ifndef STRUCTS_PRESSURE_CONTROL_H
#define STRUCTS_PRESSURE_CONTROL_H

#include "can_pressure_sensor.hpp"
#include "logger.h"
#include <memory>

namespace

Driver::Sensors {

struct Pressure
{
    std::shared_ptr<sensors::pressure::CanPressureSensor> flotation_pressure = nullptr;
    std::shared_ptr<sensors::pressure::CanPressureSensor> separation_pressure = nullptr;
};

extern Pressure pressure;

} // namespace Driver::Sensors

#endif