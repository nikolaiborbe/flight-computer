#include "Utilities/convert.hpp"
#include "Context.hpp"

namespace Utils {
void get_current_sensordata(Telemetry::GroundStation::SensorData& data)
{
    data.gnss_data     = System::data.gnss_data.update_and_read();
    data.bme_data      = System::data.bme_data.update_and_read();
    data.ms_data       = System::data.ms_data.update_and_read();
    data.icm_data      = System::data.icm_data.update_and_read();
    data.mmc_data      = System::data.mmc_data.update_and_read();
    data.stim_data     = System::data.stim_data.update_and_read();
    data.pressure_data = System::data.pressure_data.update_and_read();
}
} // namespace Utils
