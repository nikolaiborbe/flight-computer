#include "Threads/Fsm/state_initilizer.h"
#include "States/open_separation_servo.hpp"

std::unordered_map<STATE_NAME, std::unique_ptr<StateTemplate>> states;

void init_states()
{
    states.emplace(
        STATE_NAME::IDLE, std::make_unique<Idle>(std::unordered_map<EVENT, STATE_NAME> {
                {EVENT::OPEN_FILL_CMD, STATE_NAME::FILLING},
                {EVENT::OPEN_FLOTATION_FILL_CMD, STATE_NAME::FLOTATION_FILLING},
                {EVENT::OPEN_SEPARATION_FILL_CMD, STATE_NAME::SEPARATION_FILLING},
                {EVENT::EXPECT_LIFTOFF, STATE_NAME::READY},
                {EVENT::OPEN_SEPARATION, STATE_NAME::OPEN_SEPARATION_SERVO},
                {EVENT::LOST_GROUNDSTATION_CONNECTION, STATE_NAME::FILLING}
        })
    );
    
    states.emplace(
        STATE_NAME::FILLING, std::make_unique<Filling>(std::unordered_map<EVENT, STATE_NAME> {
            {EVENT::IDLE, STATE_NAME::IDLE}
        })
    );

    states.emplace(
        STATE_NAME::FLOTATION_FILLING, std::make_unique<FlotationFilling>(std::unordered_map<EVENT, STATE_NAME> {
            {EVENT::IDLE, STATE_NAME::IDLE},
            {EVENT::LOST_GROUNDSTATION_CONNECTION, STATE_NAME::FILLING}
        })
    );

    states.emplace(
        STATE_NAME::SEPARATION_FILLING, std::make_unique<SeparationFilling>(std::unordered_map<EVENT, STATE_NAME> {
            {EVENT::IDLE, STATE_NAME::IDLE},
            {EVENT::LOST_GROUNDSTATION_CONNECTION, STATE_NAME::FILLING}
        })
    );

    states.emplace(STATE_NAME::OPEN_SEPARATION_SERVO, std::make_unique<open_separation_servo>(std::unordered_map<EVENT, STATE_NAME> {
        {EVENT::IDLE, STATE_NAME::IDLE}
    }));

    states.emplace(
        STATE_NAME::READY, std::make_unique<Ready>(std::unordered_map<EVENT, STATE_NAME> {
            {EVENT::THRUSTING_DETECTED, STATE_NAME::THRUSTING},
            {EVENT::IDLE, STATE_NAME::IDLE},
            {EVENT::LOST_GROUNDSTATION_CONNECTION, STATE_NAME::FILLING}
        })
    );

    states.emplace(
        STATE_NAME::THRUSTING, std::make_unique<Thrusting>(std::unordered_map<EVENT, STATE_NAME> {
            {EVENT::THRUSTING_FINISHED, STATE_NAME::COASTING},
            {EVENT::IDLE, STATE_NAME::IDLE}
        })
    );

    states.emplace(
        STATE_NAME::COASTING, std::make_unique<Coasting>(std::unordered_map<EVENT, STATE_NAME> {
            {EVENT::APOGEE_DETECTED, STATE_NAME::SEPARATION}
        })
    );

    states.emplace(
        STATE_NAME::SEPARATION, std::make_unique<Separation>(std::unordered_map<EVENT, STATE_NAME> {
            {EVENT::FLOTATION, STATE_NAME::FLOTATION}
        })
    );

    states.emplace(
        STATE_NAME::FLOTATION, std::make_unique<Flotation>(std::unordered_map<EVENT, STATE_NAME> {
            {EVENT::SHUTDOWN, STATE_NAME::SHUTDOWN}
        })
    );

    states.emplace(
        STATE_NAME::SHUTDOWN, std::make_unique<Shutdown>(std::unordered_map<EVENT, STATE_NAME> {
            {EVENT::IDLE, STATE_NAME::IDLE}
        })
    );

}
