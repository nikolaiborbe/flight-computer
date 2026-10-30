#pragma once

#include <cstdint>
#include <cmath>

namespace units {

template<typename IntT>
IntT float_to_fixed(float x, float scale) {
	const float y = std::round(x * scale);

	if (y > static_cast<float>(std::numeric_limits<IntT>::max()))
		return std::numeric_limits<IntT>::max();
	if (y < static_cast<float>(std::numeric_limits<IntT>::min()))
		return std::numeric_limits<IntT>::min();

	return static_cast<IntT>(y);
}

template<typename IntT>
float fixed_to_float(IntT x, float scale) {
	return static_cast<float>(x) / scale;
}

inline int16_t deg_to_cdeg(float deg) {
	return float_to_fixed<int16_t>(deg, 100.0f);
}

inline float cdeg_to_deg(int16_t cdeg) {
	return fixed_to_float(cdeg, 100.0f);
}

inline int16_t barToCbar(float bar) {
	return float_to_fixed<int16_t>(bar, 100.0f);
}

inline float cbarToBar(int16_t cbar) {
	return fixed_to_float(cbar, 100.0f);
}

inline int16_t kg_to_dkg(float kg) {
	return float_to_fixed<int16_t>(kg, 10.0f);
}

inline float dkg_to_kg(int16_t dkg) {
	return fixed_to_float(dkg, 10.0f);
}

}
