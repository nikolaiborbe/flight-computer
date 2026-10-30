#include "BatteryMonitor/can_battery_monitor.hpp"

#include <cstring>

namespace sensors::battery {

CanBatteryMonitor::CanBatteryMonitor(PCan& pcan, pcan_node_id_t node_id)
	: pcan(pcan), endpoint(PCAN_ENDPOINT_ID(node_id, 0)) {
	voltage.publish(0.0f);
}

int CanBatteryMonitor::init() {
	return pcan.subscribe_telemetry(endpoint, this, &CanBatteryMonitor::on_telemetry);
}

float CanBatteryMonitor::get_voltage() const {
	return voltage.read();
}

void CanBatteryMonitor::on_telemetry(pcan_message_t* msg) {
	if (!msg || msg->channel_id != endpoint.channel_id || msg->size < sizeof(float)) {
		return;
	}

	float v = 0.0f;
	std::memcpy(&v, msg->data, sizeof(v));
	voltage.publish(v);
}

} 
