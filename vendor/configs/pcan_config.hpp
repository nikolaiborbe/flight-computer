#ifndef NAMESPACE_FC_CONFIGS_PCAN_HPP
#define NAMESPACE_FC_CONFIGS_PCAN_HPP

#include "pcan_threadx.hpp"
#include "pcan_core.hpp"

namespace FC::Configs::PCan
{
	using ThreadX = PCanThreadX<{
		.max_subscription_count = 32,
        .nominal_speed = PCAN_SPEED_CONFIG_1_MBIT,
        .data_speed = PCAN_SPEED_CONFIG_2_MBIT,
        .bit_rate_switching_on = true
	}>;
}

#endif // NAMESPACE_FC_CONFIGS_PCAN_HPP
