#include "App.hpp"
#include "Actuators.hpp"
#include "Data/Writers.hpp"
#include "Utilities/utilities_buzzer.h"
#include "can_pressure_sensor.hpp"
#include "fc_ids.h"
#include "fdcan.h"
#include "fsm_pressure.h"
#include "pcan_config.hpp"
#include "sdmmc.h"
#include "services/BMS_service.hpp"
#include "tim.h"
#include "tx_api.h"
#include <memory>

FC::Configs::PCan::ThreadX App::pcan1_instance(&hfdcan1, FC_MASTER_NODE);
FC::Configs::PCan::ThreadX App::pcan2_instance(&hfdcan2, FC_MASTER_NODE);
Network::NetworkManager App::network_instance(NetXDuoEthIpInstance, NxAppPool);
Network::BroadcastManager App::broadcast_manager_instance(App::network_instance);

Network::Services::HeartbeatService App::heartbeat_service;
Network::Services::SensorService App::sensor_service;
Network::Services::KalmanService App::kalman_service;
Network::Services::StatusService App::status_service;
Network::Services::BMSService App::bms_service;

SdCardManager __attribute__((section(".StorageMemory"))) App::sd_card_manager_instance(
    &hsd1, STORAGE_CD_GPIO_Port, STORAGE_CD_Pin, "SD CARD MANAGER", "FC"
);
FileLogger __attribute__((section(".StorageMemory"))) App::sd_card_logger_instance(
    App::sd_card_manager_instance, "SD CARD LOGGER", "telemetry.bin", LED_STOR_GPIO_Port,
    LED_STOR_Pin
);

Network::CommandManager App::command_manager_instance(App::network_instance);

extern TIM_HandleTypeDef htim15;

static_assert(sizeof(ULONG) >= sizeof(void*), "thread_input cannot hold a pointer");

void App::setup()
{
    bool res = true;
    System::reduce_network_output_writer.store(true);
    res &= setup_data_context();
    res &= setup_pcan_context();
    res &= setup_network_context();
    res &= setup_storage_context();
    res &= setup_broadcast_context();
    res &= setup_command_context();
    res &= setup_controllers();
    res &= setup_pressure();

    EventMachine::init();

    while (!res)
    {
        play_failure_sound();
    }

    res &= setup_threads();
    init_states();

    play_success_sound();
}

bool App::setup_data_context()
{
    const bool res = Telemetry::bind_readers();

#ifdef FC_PYTHON_TESTING
    command_manager_instance.test_state_service().attach_writers(&Telemetry::Writers::test_feed);
#endif

    return res;
}

bool App::setup_pcan_context()
{
    static PCanBlinky blinky1({LED_FDCAN1_GPIO_Port, LED_FDCAN1_Pin});
    static PCanBlinky blinky2({LED_FDCAN2_GPIO_Port, LED_FDCAN2_Pin});

    pcan1_instance.register_plugin(blinky1);
    pcan2_instance.register_plugin(blinky2);
    
    pcan1_instance.init();
    pcan2_instance.init();

    System::pcan1 = std::shared_ptr<PCan>(&pcan1_instance, [](auto*){});
    System::pcan2 = std::shared_ptr<PCan>(&pcan2_instance, [](auto*){});
    return true;
}

bool App::setup_network_context()
{

    const unsigned int st = network_instance.start();

    if (st != NX_SUCCESS && st != TX_SUCCESS)
        return false;

    Network::manager = std::shared_ptr<Network::NetworkManager>(&network_instance, [](auto*){});
    return true;
}

bool App::setup_storage_context()
{
    System::storage_manager = std::shared_ptr<StorageManager>(&sd_card_manager_instance, [](auto*){});
    System::file_logger = std::shared_ptr<FileLogger>(&sd_card_logger_instance, [](auto*){});

    const UINT manager_status = sd_card_manager_instance.init();
    const UINT logger_status = sd_card_logger_instance.init();

    return manager_status == TX_SUCCESS && logger_status == TX_SUCCESS;
}

bool App::setup_broadcast_context()
{
    bool res = true;

    res &= broadcast_manager_instance.register_service(heartbeat_service);
    res &= broadcast_manager_instance.register_service(sensor_service);
    res &= broadcast_manager_instance.register_service(kalman_service);
    res &= broadcast_manager_instance.register_service(status_service);
    res &= broadcast_manager_instance.register_service(bms_service);

    broadcast_manager_instance.attach_logger(sd_card_logger_instance);

    Network::broadcaster
        = std::shared_ptr<Network::BroadcastManager>(&broadcast_manager_instance, [](auto*){});
    return res;
}

bool App::setup_command_context()
{
    const bool res = command_manager_instance.start();

    Network::commands
        = std::shared_ptr<Network::CommandManager>(&command_manager_instance, [](auto*){});
    return res;
}

bool App::setup_controllers()
{
    if (System::pcan1) {
        Driver::Actuators::init(*System::pcan1);
    }

    return true;
}

bool App::setup_pressure()
{
    Driver::Sensors::pressure.flotation_pressure =
        std::make_shared<sensors::pressure::CanPressureSensor>(
            *System::pcan2, FC_PRESSURE_FLOTATION_ENDPOINT);

    Driver::Sensors::pressure.separation_pressure =
        std::make_shared<sensors::pressure::CanPressureSensor>(
            *System::pcan2, FC_PRESSURE_SEPARATION_ENDPOINT);

    return true;
}

bool App::setup_threads()
{
    for (const Thread::Type& thread : Thread::Threads)
    {
        const UINT res = tx_thread_create(
            thread.thread, 
            const_cast<char*>(thread.name), 
            thread.entry, 
            thread.input, 
            thread.stack_ptr, 
            thread.stack_size, 
            thread.priority, 
            thread.preempt_threshold, 
            thread.time_slice, 
            thread.auto_start
        );

        if (res != TX_SUCCESS)
            return res;
    }

    return true;
}

extern "C" void app_setup() { App::setup(); }
