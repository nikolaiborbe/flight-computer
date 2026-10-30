#ifndef CLASS_APP_HPP
#define CLASS_APP_HPP

/*
 * Singulare entry point to initalize FC
 */

// App
#include "Context.hpp"
#include "Threads/threads.hpp"
#include "Ids/fc_ids.h"
#include "pcan_threadx.hpp"
#include "pcan_core.hpp"
#include "pcan_blinky.hpp"
#include "Networking/broadcast_manager.hpp"
#include "Networking/services/heartbeat_service.hpp"
#include "Networking/services/sensor_service.hpp"
#include "Networking/services/kalman_service.hpp"
#include "Networking/services/status_service.hpp"
#include "Networking/command_manager.hpp"
#include "Networking/network_context.hpp"
#include "Networking/services/BMS_service.hpp"

// vendor
#include "configs/pcan_config.hpp"
#include "file_logger.hpp"
#include "sd_card_manager.hpp"
#include "main.h"

extern "C"
{
    extern NX_IP NetXDuoEthIpInstance;
    extern NX_PACKET_POOL NxAppPool;
}

#ifdef __cplusplus
extern "C" {
#endif
    void app_setup(void);
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
class App
{
private:
    // PCan
    static FC::Configs::PCan::ThreadX pcan1_instance;
    static FC::Configs::PCan::ThreadX pcan2_instance;

    // Network managers
    static Network::NetworkManager network_instance;
    static Network::BroadcastManager broadcast_manager_instance;
    static Network::CommandManager command_manager_instance;

    // Network services
    static Network::Services::HeartbeatService heartbeat_service;
    static Network::Services::SensorService sensor_service;
    static Network::Services::KalmanService kalman_service;
    static Network::Services::StatusService status_service;
    static Network::Services::BMSService bms_service;

    // Storage
    static SdCardManager sd_card_manager_instance;
    static FileLogger sd_card_logger_instance;

public:
    static void setup();

private:
	// setups
    static bool setup_data_context();
    static bool setup_pcan_context();
    static bool setup_network_context();
    static bool setup_storage_context();
    static bool setup_broadcast_context();
    static bool setup_command_context();
    static bool setup_controllers();
    static bool setup_pressure();
    static bool setup_threads();
    
private:
    // sound
    static void notify_setup_succeded();
	static void notify_setup_failed();
};
#endif // __cplusplus
#endif // CLASS_APP_HPP
