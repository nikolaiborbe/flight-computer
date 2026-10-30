#ifndef CONTEXT_HPP
#define CONTEXT_HPP

// App
#include "event_machine.h"
#include "pcan_core.hpp"
#include "Data/Local.hpp"
#include "file_logger.hpp"
#include "storage_manager.hpp"

// std
#include <memory>


/*
 *  GLOBAL system context
 */
namespace System
{
    extern algo::Seqlock<STATE_NAME>::Reader current_state;

    extern std::shared_ptr<PCan> pcan1;
    extern std::shared_ptr<PCan> pcan2;

    extern std::shared_ptr<StorageManager> storage_manager;
    extern std::shared_ptr<FileLogger> file_logger;

    extern Telemetry::Local::Sensors data;
    extern Telemetry::Local::Status status;
    extern algo::Seqlock<Telemetry::GroundStation::KalmanData>::Reader kalman_data;

    // Yeah this is dogshit
    extern algo::Seqlock<bool> reduce_network_seqlock;
    extern algo::Seqlock<bool>::Writer reduce_network_output_writer;
    extern algo::Seqlock<bool>::Reader reduce_network_output_reader;

    // True while the ground station heartbeat keeps arriving (every ~15 s),
    // false once one is missed. Read via ground_station_heartbeat.update_and_read().
    extern algo::Seqlock<bool> ground_station_heartbeat_seqlock;
    extern algo::Seqlock<bool>::Writer ground_station_heartbeat_writer;
    extern algo::Seqlock<bool>::Reader ground_station_heartbeat;

    extern algo::Seqlock<bool> suicide;
};

#endif