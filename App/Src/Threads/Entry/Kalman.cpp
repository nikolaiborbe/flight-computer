#include "Threads/Entry/Kalman.hpp"
#include "Context.hpp"
#include "Data/Writers.hpp"
#include "Datastructures/Seqlock.hpp"
#include "Utilities/math.hpp"
#include "main.h"
#include <limits>

namespace {
    constexpr double LAUNCH_PAD_ALTITUDE_M = 9.9;
    constexpr uint32_t BAROMETER_CALIBRATION_TIME_MS = 2000;

    struct BarometerReference {
        bool collecting = false;
        bool locked = false;
        ULONG start_ticks = 0;
        uint32_t samples = 0;
        double temperature_sum_c = 0.0;
        double pressure_sum_hpa = 0.0;
        double temperature_c = 0.0;
        double pressure_hpa = 0.0;

        void reset_collection()
        {
            collecting = false;
            samples = 0;
            temperature_sum_c = 0.0;
            pressure_sum_hpa = 0.0;
        }

        void start_collection(const ULONG current_time)
        {
            reset_collection();
            collecting = true;
            start_ticks = current_time;
        }

        bool update(
            const Telemetry::GroundStation::MS5611_DATA& ms,
            const STATE_NAME current_state,
            const ULONG current_time
        )
        {
            if (locked) {
                return true;
            }

            if (current_state != STATE_NAME::READY) {
                reset_collection();
                return false;
            }

            if (!collecting) {
                start_collection(current_time);
            }

            if (valid_sample(ms)) {
                temperature_sum_c += ms.temperature;
                pressure_sum_hpa += ms.pressure;
                ++samples;
            }

            if ((current_time - start_ticks) < ms_to_ticks(BAROMETER_CALIBRATION_TIME_MS)) {
                return false;
            }

            if (samples == 0) {
                start_collection(current_time);
                return false;
            }

            const double count = static_cast<double>(samples);
            temperature_c = temperature_sum_c / count;
            pressure_hpa = pressure_sum_hpa / count;
            locked = true;
            return true;
        }

        static bool valid_sample(const Telemetry::GroundStation::MS5611_DATA& ms)
        {
            return ms.valid
                && ms.pressure > 0.0f
                && std::isfinite(ms.pressure)
                && std::isfinite(ms.temperature);
        }
    };

    double pressure_to_calibrated_altitude(
        const float pressure_hpa,
        const BarometerReference& reference
    )
    {
        if (!reference.locked) {
            return std::numeric_limits<double>::quiet_NaN();
       }

        return Utils::pressure_to_altitude(
            pressure_hpa,
            reference.temperature_c,
            reference.pressure_hpa
        ) + LAUNCH_PAD_ALTITUDE_M;
    }
}

namespace Thread
{
    namespace Entry
    {
        void Kalman(unsigned long thread_input)
        {
            Telemetry::Writers::Kalman& writers
                = *reinterpret_cast<Telemetry::Writers::Kalman*>(thread_input);
            writers.kalman_thread_online.store(true);

            BarometerReference barometer_reference {};
            double y[4];
            while (true)
            {
                const ULONG current_time = tx_time_get();
                const STATE_NAME current_state = System::current_state.update_and_read();
                const Telemetry::GroundStation::MS5611_DATA& ms = System::data.ms_data.update_and_read();
                const Telemetry::Shared::ImuData& icm = System::data.icm_data.update_and_read();

                barometer_reference.update(ms, current_state, current_time);

                // MS5611_DATA::pressure is in mbar.
                const double alt = pressure_to_calibrated_altitude(ms.pressure, barometer_reference);
                if (!std::isfinite(alt)) {
                    tx_thread_sleep(KALMAN_THREAD_SLEEP_TIMER);
                    continue;
                }

                const double acc = icm.acc.acc_z_m_per_ss;

                constexpr double R = 200.0;
                kalmanfilter(alt, acc, 0.005, 0.005, 0.00005, R, 10, 2000, 0.01, 1, 0, y);

                writers.kalman_data.store(Telemetry::GroundStation::KalmanData{ y[0], y[1] });

                tx_thread_sleep(KALMAN_THREAD_SLEEP_TIMER);
            }
            writers.kalman_thread_online.store(false);
        }
    }
}
