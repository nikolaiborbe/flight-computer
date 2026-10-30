#include "Utilities/math.hpp"

namespace Utils 
{
	float pressure_to_altitude(float pressure_hpa, float starting_temperature_celsius, float starting_pressure_hpa)
	{
		constexpr float kelvin_offset = 273.15f;
        constexpr float temperature_lapse_rate_k_per_m = 0.0065f;
        constexpr float barometric_exponent = 0.1902631f;

        if (pressure_hpa <= 0.0f || starting_pressure_hpa <= 0.0f) {
            return std::numeric_limits<float>::quiet_NaN();
        }

        const float starting_temperature_kelvin = starting_temperature_celsius + kelvin_offset;
        const float pressure_ratio = pressure_hpa / starting_pressure_hpa;

        return (starting_temperature_kelvin / temperature_lapse_rate_k_per_m) *
               (1.0f - std::pow(pressure_ratio, barometric_exponent));

	}
}
