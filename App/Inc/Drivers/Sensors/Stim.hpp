#ifndef NAMESPACE_SENSOR_STIM_HPP
#define NAMESPACE_SENSOR_STIM_HPP

#include "Data/Local.hpp"
#include "IMU.h"
#include "usart.h"

namespace Driver::Sensors
{
    namespace STIM
    {
		constexpr Orientation orientation = Orientation{0.f, static_cast<float>(PI / 2.0), 0.f};
        struct StimData
        {
            bool valid;
            Telemetry::GroundStation::STIM377H_DATA value;
        };

        class Stim
        {
        public:
            explicit Stim(UART_HandleTypeDef* uart);
            StimData read();

        private:
            UART_HandleTypeDef* uart_;
            uint8_t frame_[38] {};
            uint16_t frame_index_ = 0;

            static int32_t read_i24_be(const uint8_t* bytes);
        };

        Stim init();
        void mount_offset(Telemetry::GroundStation::STIM377H_DATA& data);
    }
}

#endif // !NAMESPACE_SENSOR_STIM_HPP
