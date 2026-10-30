#include <cmath>

/**
 * Calculates relative permiability for a cylindrical capacitor.
 */
float e_r(float c_ref);

/**
 * Extrapolates height using capacitance value of a submerged cylindrical capacitor.
 */
float h(float c_total, float e_rg, float e_rl);

/**
 * Computes liquid level height from the main, gas-reference and liquid-reference
 * capacitance measurements.
 */
float height(float c_main, float c_env_gas, float c_env_liquid);
