#include "Context.hpp"

namespace System
{
    algo::Seqlock<STATE_NAME>::Reader current_state;
    
    std::shared_ptr<PCan> pcan1 = nullptr;
    std::shared_ptr<PCan> pcan2 = nullptr;

    std::shared_ptr<StorageManager> storage_manager = nullptr;
    std::shared_ptr<FileLogger> file_logger = nullptr;

    Telemetry::Local::Sensors data;
    Telemetry::Local::Status status {};
    algo::Seqlock<Telemetry::GroundStation::KalmanData>::Reader kalman_data;

    algo::Seqlock<bool> reduce_network_seqlock;
    algo::Seqlock<bool>::Writer reduce_network_output_writer;
    algo::Seqlock<bool>::Reader reduce_network_output_reader;

    algo::Seqlock<bool> ground_station_heartbeat_seqlock;
    algo::Seqlock<bool>::Writer ground_station_heartbeat_writer;
    algo::Seqlock<bool>::Reader ground_station_heartbeat;

    algo::Seqlock<bool> suicide;
}
