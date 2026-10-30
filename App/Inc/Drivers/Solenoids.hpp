#ifndef NAMESPACE_SOLENOIDS_HPP
#define NAMESPACE_SOLENOIDS_HPP

// vendor
#include "Ids/fc_ids.h"
#include "can_solenoid.hpp"

namespace Driver {

struct Solenoid {
    actuators::solenoid::CanSolenoid separation_filling;
    actuators::solenoid::CanSolenoid flotation_filling;
    actuators::solenoid::CanSolenoid flotation; // NO-solenoid

    explicit Solenoid(PCan& pcan)
        : separation_filling(pcan, pcan_endpoint_id_t {FC_SOLENOID_SEPARATION_FILLING})
        , flotation_filling(pcan, pcan_endpoint_id_t {FC_SOLENOID_FLOTATION_FILLING})
        , flotation(pcan, pcan_endpoint_id_t {FC_SOLENOID_FLOTATION})
    {
    }
};

} // namespace Driver

#endif