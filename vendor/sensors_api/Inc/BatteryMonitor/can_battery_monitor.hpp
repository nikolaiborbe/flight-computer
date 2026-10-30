#pragma once

#include "pcan_core.hpp"
#include "seqlock.h"

namespace sensors::battery {

class CanBatteryMonitor {
public:
	explicit CanBatteryMonitor(PCan& pcan, pcan_node_id_t node_id);

    int init();

	float get_voltage() const;

private:
	void on_telemetry(pcan_message_t* msg);

    PCan pcan;
	pcan_endpoint_id_t endpoint;
	SeqLock<float> voltage{};
};

} 
