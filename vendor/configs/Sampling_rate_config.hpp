#ifndef NAMESPACE_FC_CONFIGS_SAMPLING_RATE_HPP
#define NAMESPACE_FC_CONFIGS_SAMPLING_RATE_HPP


#include "cstdint"
#define RTOS_HZ_TO_TICKS(hz) ((TX_TIMER_TICKS_PER_SECOND + hz - 1u) / (hz))

namespace FC::Config::SamplingRate
{
	constexpr uint16_t FSM_HZ  = 200;
	constexpr uint16_t NETWORK_HZ = 200;
	constexpr uint16_t SENSOR_HZ = 500;
	constexpr uint16_t KALMAN_HZ = 100;
	constexpr uint16_t PCAN_HZ = 1000;
}


#endif
