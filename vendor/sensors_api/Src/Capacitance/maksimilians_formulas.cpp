#include "maksimilians_formulas.hpp"

// These values are from "Untitled document" by Berg and Melø.
// UTL: https://docs.google.com/document/d/1zaEb0IEq5VUNLzsydXWxnjpmxi2FMK8v2uADBbXInos/

constexpr float a_l = 0.001500;
constexpr float a_r = 0.001500;
constexpr float b_l = 0.007874;
constexpr float b_r = 0.002030;
constexpr float l_l = 1.11;
constexpr float l_r = 0.07;
constexpr float e_0 = 8.85e-12;

// These formulas are taken from section 11.8.1.2 in "Propulsion Design Document CDR 2026".
// URL: https://docs.google.com/document/d/17vgUx2-GvhKUNj4Lx3nrBsjNrnK6_iW2zGYKogI0XuM/

float e_r(float c_ref) {
    return (c_ref*std::log(b_r/a_r))/(2*M_PI*e_0*l_r);
}

float h(float c_total, float e_rg, float e_rl) {
    return (std::log(b_l/a_l)/(2*M_PI*e_0)*c_total - e_rg*l_l)/(e_rl - e_rg);
}

float height(float c_main, float c_env_gas, float c_env_liquid) {
    return h(c_main, e_r(c_env_gas), e_r(c_env_liquid));
}
