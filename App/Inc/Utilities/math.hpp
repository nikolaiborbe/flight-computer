#pragma once
#include <cmath>

namespace Utils
{
	float pressure_to_altitude(float pressure_hpa, float starting_temperature_celsius, float starting_pressure_hpa);

class RK3_Derivative {
public:
    float prevError;
    float prevDeriv;
    RK3_Derivative() : prevError(0.0f), prevDeriv(0.0f) {}
    float compute(float error, float dt)
    {
        float k1 = (error - prevError) / dt;
        float k2 = ((error + 0.5f * dt * k1) - prevError) / dt;
        float k3 = ((error - dt * k1 + 2.0f * dt * k2) - prevError) / dt;

        float deriv = prevDeriv + (dt / 6.0f) * (k1 + 4.0f * k2 + k3);

        prevError = error;
        prevDeriv = deriv;

        return deriv;
    }
}; 
}
