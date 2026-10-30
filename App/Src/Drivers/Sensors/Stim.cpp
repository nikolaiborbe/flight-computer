#include "Drivers/Sensors/Stim.hpp"

namespace Driver::Sensors
{
    namespace STIM
    {
        namespace
        {
            constexpr uint8_t datagram_id = 0x93;
            constexpr uint16_t datagram_size = 38;
            constexpr uint16_t crc_offset = datagram_size - 4;
            constexpr uint8_t crc_dummy_bytes = 2;
            constexpr uint16_t rx_ring_size = 256;
            constexpr uint16_t rx_ring_mask = rx_ring_size - 1;
            constexpr float standard_gravity = 9.80665f;
            constexpr float acc_lsb_per_g = static_cast<float>(1u << 18);

            static_assert((rx_ring_size & rx_ring_mask) == 0u);

            UART_HandleTypeDef* rx_uart = nullptr;
            uint8_t rx_byte = 0;
            uint8_t rx_ring[rx_ring_size] {};
            volatile uint16_t rx_head = 0;
            volatile uint16_t rx_tail = 0;

            uint32_t crc32_stim_update(uint32_t crc, uint8_t byte)
            {
                crc ^= static_cast<uint32_t>(byte) << 24;

                for (uint8_t bit = 0; bit < 8; ++bit) {
                    if ((crc & 0x80000000u) != 0u) {
                        crc = (crc << 1) ^ 0x04C11DB7u;
                    } else {
                        crc <<= 1;
                    }
                }

                return crc;
            }

            uint32_t read_u32_be(const uint8_t* bytes)
            {
                return (static_cast<uint32_t>(bytes[0]) << 24)
                    | (static_cast<uint32_t>(bytes[1]) << 16)
                    | (static_cast<uint32_t>(bytes[2]) << 8)
                    | static_cast<uint32_t>(bytes[3]);
            }

            bool has_valid_crc(const uint8_t* rx)
            {
                uint32_t crc = 0xFFFFFFFFu;

                for (uint16_t i = 0; i < crc_offset; ++i) {
                    crc = crc32_stim_update(crc, rx[i]);
                }

                for (uint8_t i = 0; i < crc_dummy_bytes; ++i) {
                    crc = crc32_stim_update(crc, 0x00);
                }

                return crc == read_u32_be(&rx[crc_offset]);
            }

            bool has_ok_status(const uint8_t* rx)
            {
                return rx[10] == 0u && rx[20] == 0u;
            }

            void clear_uart_errors(UART_HandleTypeDef* uart)
            {
                __HAL_UART_CLEAR_OREFLAG(uart);
                __HAL_UART_CLEAR_FEFLAG(uart);
                __HAL_UART_CLEAR_NEFLAG(uart);
                __HAL_UART_CLEAR_PEFLAG(uart);
                uart->ErrorCode = HAL_UART_ERROR_NONE;
            }

            void restart_rx()
            {
                if (rx_uart == nullptr) {
                    return;
                }

                (void)HAL_UART_Receive_IT(rx_uart, &rx_byte, 1);
            }

            void start_rx(UART_HandleTypeDef* uart)
            {
                rx_uart = uart;
                rx_head = 0;
                rx_tail = 0;
                clear_uart_errors(uart);
                restart_rx();
            }

            void push_rx_byte(uint8_t byte)
            {
                const uint16_t head = rx_head;
                const uint16_t next = static_cast<uint16_t>((head + 1u) & rx_ring_mask);

                if (next == rx_tail) {
                    rx_tail = static_cast<uint16_t>((rx_tail + 1u) & rx_ring_mask);
                }

                rx_ring[head] = byte;
                rx_head = next;
            }

            bool pop_rx_byte(uint8_t& byte)
            {
                const uint16_t tail = rx_tail;
                if (tail == rx_head) {
                    return false;
                }

                byte = rx_ring[tail];
                rx_tail = static_cast<uint16_t>((tail + 1u) & rx_ring_mask);
                return true;
            }

            void accel_mount_offset(Telemetry::Shared::AccelerationData& data)
            {
                ::AccelerationData mounted {
                    .acc_x = static_cast<float>(data.acc_x_m_per_ss),
                    .acc_y = static_cast<float>(data.acc_y_m_per_ss),
                    .acc_z = static_cast<float>(data.acc_z_m_per_ss),
                };

                IMU_methods::accel_mount_offset(mounted, orientation);

                data.acc_x_m_per_ss = static_cast<float>(mounted.acc_x);
                data.acc_y_m_per_ss = static_cast<float>(mounted.acc_y);
                data.acc_z_m_per_ss = static_cast<float>(mounted.acc_z);
            }

            void gyro_mount_offset(Telemetry::Shared::GyroData& data)
            {
                ::GyroData mounted {
                    .gyro_x = static_cast<float>(data.gyro_x),
                    .gyro_y = static_cast<float>(data.gyro_y),
                    .gyro_z = static_cast<float>(data.gyro_z),
                };

                IMU_methods::gyro_mount_offset(mounted, orientation);

                data.gyro_x = static_cast<float>(mounted.gyro_x);
                data.gyro_y = static_cast<float>(mounted.gyro_y);
                data.gyro_z = static_cast<float>(mounted.gyro_z);
            }
        }

        Stim::Stim(UART_HandleTypeDef* uart)
            : uart_(uart)
        {
            start_rx(uart_);
        }

        StimData Stim::read()
        {
            uint8_t byte = 0;
            while (pop_rx_byte(byte)) {
                if (frame_index_ == 0u && byte != datagram_id) {
                    continue;
                }

                frame_[frame_index_++] = byte;

                if (frame_index_ < datagram_size) {
                    continue;
                }

                frame_index_ = 0;

                if (!has_valid_crc(frame_) || !has_ok_status(frame_)) {
                    continue;
                }

                StimData data {};
                data.valid = true;

                data.value.gyro.gyro_x = static_cast<float>(read_i24_be(&frame_[1])) / 16384.0f;
                data.value.gyro.gyro_y = static_cast<float>(read_i24_be(&frame_[4])) / 16384.0f;
                data.value.gyro.gyro_z = static_cast<float>(read_i24_be(&frame_[7])) / 16384.0f;

                data.value.acc.acc_x_m_per_ss
                    = static_cast<float>(read_i24_be(&frame_[11])) * standard_gravity / acc_lsb_per_g;
                data.value.acc.acc_y_m_per_ss
                    = static_cast<float>(read_i24_be(&frame_[14])) * standard_gravity / acc_lsb_per_g;
                data.value.acc.acc_z_m_per_ss
                    = static_cast<float>(read_i24_be(&frame_[17])) * standard_gravity / acc_lsb_per_g;

                return data;
            }

            return {};
        }

        int32_t Stim::read_i24_be(const uint8_t* bytes)
        {
            const uint32_t raw = (static_cast<uint32_t>(bytes[0]) << 16)
                | (static_cast<uint32_t>(bytes[1]) << 8)
                | static_cast<uint32_t>(bytes[2]);

            if ((raw & 0x800000u) != 0u) {
                return static_cast<int32_t>(raw) - 0x1000000;
            }

            return static_cast<int32_t>(raw);
        }

        Stim init()
        {
            return Stim(&huart7);
        }

        void mount_offset(Telemetry::GroundStation::STIM377H_DATA& data)
        {
            accel_mount_offset(data.acc);
            gyro_mount_offset(data.gyro);
        }
    }
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart != Driver::Sensors::STIM::rx_uart) {
        return;
    }

    Driver::Sensors::STIM::push_rx_byte(Driver::Sensors::STIM::rx_byte);
    Driver::Sensors::STIM::restart_rx();
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart)
{
    if (huart != Driver::Sensors::STIM::rx_uart) {
        return;
    }

    Driver::Sensors::STIM::clear_uart_errors(huart);
    Driver::Sensors::STIM::restart_rx();
}
