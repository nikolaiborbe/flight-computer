#include "Threads/Entry/Sensor.hpp"
#include "Context.hpp"
#include "Data/Writers.hpp"
#include "Datastructures/Seqlock.hpp"
#include "Drivers/Sensors/Icm.hpp"
#include "Drivers/Sensors/Mmc.hpp"
#include "Drivers/Sensors/Stim.hpp"
#include "ICM40609D.h"
#include "IMU.h"
#include "MMC5983MA.h"
#include "Utilities/config.h"
#include "fc_ids.h"
#include "spi.h"

namespace Thread::Entry {
void Sensor(unsigned long thread_input)
{
#ifndef FC_PYTHON_TESTING
    // currently not in context because of include conflicts
    // fix sensor repo

    // The unique write handles for this thread's cells, claimed in App.cpp
    // and handed over through thread_input.
    Telemetry::Writers::Sensor& writers = *reinterpret_cast<Telemetry::Writers::Sensor*>(thread_input);

    // init drivers
    ICM40609D icm = Driver::Sensors::ICM::init();
    MMC5983MA mmc = Driver::Sensors::MMC::init();
    Driver::Sensors::STIM::Stim stim = Driver::Sensors::STIM::init();

    ::ImuData icm_data{};
    ::Mmc5983maData mmc_data{};
    writers.sensor_thread_online.store(true);
    while (true) 
    {

        // Read data
        icm.read(icm_data);
        mmc.read(mmc_data);
        Driver::Sensors::MMC::linear_transform_mag(mmc_data);

        Driver::Sensors::STIM::StimData stim_data = stim.read();
        if (stim_data.valid) {
            Driver::Sensors::STIM::mount_offset(stim_data.value);
            writers.stim_data.store(stim_data.value);
        }

		IMU_methods::accel_mount_offset(icm_data.acc, Driver::Sensors::ICM::orientation);
		IMU_methods::accel_mount_offset(icm_data.acc, Driver::Sensors::ICM::orientation2);

		IMU_methods::gyro_mount_offset(icm_data.gyro, Driver::Sensors::ICM::orientation);
		IMU_methods::gyro_mount_offset(icm_data.gyro, Driver::Sensors::ICM::orientation2);

        IMU_methods::vector_mount_offset(
            mmc_data.magnetic_gauss.x, 
            mmc_data.magnetic_gauss.y, 
            mmc_data.magnetic_gauss.z, 
            Driver::Sensors::MMC::orientation);


        // Update seqlock data
        Telemetry::Shared::ImuData icm_shared {
            icm_data.temperature.temperature,
            { icm_data.gyro.gyro_x, icm_data.gyro.gyro_y, icm_data.gyro.gyro_z },
            { icm_data.acc.acc_x, icm_data.acc.acc_y, icm_data.acc.acc_z },
            icm_data.timestamp
        };

        Telemetry::Shared::MagnetometerData mmc_shared {
            mmc_data.magnetic_gauss.x,
            mmc_data.magnetic_gauss.y,
            mmc_data.magnetic_gauss.z,
        };

        writers.icm_data.store(icm_shared);
        writers.mmc_data.store(mmc_shared);

        // Update statuses
        writers.icm_online.store(icm_data.valid);
        writers.mmc_online.store(mmc_data.valid);
        writers.stim_online.store(stim_data.valid);

        tx_thread_sleep(SENSOR_THREAD_SLEEP_TIMER);
    }

    writers.sensor_thread_online.store(false);
#endif
}
}
