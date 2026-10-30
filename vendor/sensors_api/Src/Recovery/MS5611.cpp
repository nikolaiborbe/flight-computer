/**
 * @file MS5611.cpp
 *
 * @author Nikolai G. Borbe (nikolai.borbe@propulsentnu.com)
 * @brief This file contains the implementation for the MS5611 (barometric pressure) driver.
 * @version 1.0
 *
 * @date 2026-03-27
 */

#include "p-timer.hpp"
#include "MS5611.h"

void MS5611::_cs_low() { HAL_GPIO_WritePin(m_spi.cs_port, m_spi.cs_pin, GPIO_PIN_RESET); }
void MS5611::_cs_high() { HAL_GPIO_WritePin(m_spi.cs_port, m_spi.cs_pin, GPIO_PIN_SET); }

LOGGER::STATUS MS5611::_send_command(u8 cmd)
{
    _cs_low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(m_spi.hspi, &cmd, 1, MS5611_SPI_TIMEOUT_MS);
    _cs_high();
    return (status == HAL_OK) ? LOGGER::STATUS::OK : LOGGER::STATUS::LOGGER_ERROR;
}

LOGGER::STATUS MS5611::_spi_cmd_receive(u8 cmd, u8* data, u16 len)
{
    _cs_low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(m_spi.hspi, &cmd, 1, MS5611_SPI_TIMEOUT_MS);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(m_spi.hspi, data, len, MS5611_SPI_TIMEOUT_MS);
    }
    _cs_high();
    return (status == HAL_OK) ? LOGGER::STATUS::OK : LOGGER::STATUS::LOGGER_ERROR;
}

LOGGER::STATUS MS5611::_soft_reset()
{
    LOGGER::STATUS status = _send_command(CMD_RESET);
    HAL_Delay(3); // PROM reload takes ~2.8 ms (datasheet page 10)
    return status;
}

LOGGER::STATUS MS5611::_read_prom(u8 addr, u16& result)
{
    u8 rx[2] = {};
    u8 cmd   = CMD_PROM_READ | (addr << 1);
    if (_spi_cmd_receive(cmd, rx, 2) != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;

    result = (static_cast<u16>(rx[0]) << 8) | rx[1];
    return LOGGER::STATUS::OK;
}

LOGGER::STATUS MS5611::_read_adc(u32& result)
{
    u8 rx[3] = {};
    if (_spi_cmd_receive(CMD_ADC_READ, rx, 3) != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;

    result = (static_cast<u32>(rx[0]) << 16) | (static_cast<u32>(rx[1]) << 8)
             | static_cast<u32>(rx[2]);
    return LOGGER::STATUS::OK;
}

u8 MS5611::_conversion_delay_ms(OSR osr)
{
    // Ceiling of max conversion times from datasheet (page 2)
    switch (osr) {
    case OSR_256: return 1;
    case OSR_512: return 2;
    case OSR_1024: return 3;
    case OSR_2048: return 5;
    case OSR_4096: return 10;
    default: return 10;
    }
}

u8 MS5611::_conv_d1_cmd(OSR osr)
{
    switch (osr) {
    case OSR_256: return CMD_CONV_D1_256;
    case OSR_512: return CMD_CONV_D1_512;
    case OSR_1024: return CMD_CONV_D1_1024;
    case OSR_2048: return CMD_CONV_D1_2048;
    case OSR_4096: return CMD_CONV_D1_4096;
    default: return CMD_CONV_D1_4096;
    }
}

u8 MS5611::_conv_d2_cmd(OSR osr)
{
    switch (osr) {
    case OSR_256: return CMD_CONV_D2_256;
    case OSR_512: return CMD_CONV_D2_512;
    case OSR_1024: return CMD_CONV_D2_1024;
    case OSR_2048: return CMD_CONV_D2_2048;
    case OSR_4096: return CMD_CONV_D2_4096;
    default: return CMD_CONV_D2_4096;
    }
}

LOGGER::STATUS MS5611::_convert_and_read(u8 conv_cmd, u32& result)
{
    if (_send_command(conv_cmd) != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;

    return _read_adc(result);
}

LOGGER::STATUS MS5611::_read_calibration_data()
{
    // PROM addresses 1-6 contain the 6 calibration coefficients (datasheet page 7)
    if (_read_prom(1, m_calib.C1) != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;
    if (_read_prom(2, m_calib.C2) != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;
    if (_read_prom(3, m_calib.C3) != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;
    if (_read_prom(4, m_calib.C4) != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;
    if (_read_prom(5, m_calib.C5) != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;
    if (_read_prom(6, m_calib.C6) != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;

    return LOGGER::STATUS::OK;
}

void MS5611::_compensate(u32 D1, u32 D2, float& temperature, float& pressure)
{
    // First order compensation (datasheet page 7)
    s32 dT   = static_cast<s32>(D2) - (static_cast<s32>(m_calib.C5) << 8);
    s32 TEMP = 2000 + static_cast<s32>((static_cast<s64>(dT) * m_calib.C6) >> 23);

    s64 OFF  = (static_cast<s64>(m_calib.C2) << 16) + ((static_cast<s64>(m_calib.C4) * dT) >> 7);
    s64 SENS = (static_cast<s64>(m_calib.C1) << 15) + ((static_cast<s64>(m_calib.C3) * dT) >> 8);

    // Second order temperature compensation (datasheet page 8)
    s32 T2    = 0;
    s64 OFF2  = 0;
    s64 SENS2 = 0;

    if (TEMP < 2000) {
        // Low temperature (< 20°C)
        T2            = static_cast<s32>((static_cast<s64>(dT) * dT) >> 31);
        s64 temp_diff = static_cast<s64>(TEMP - 2000);
        OFF2          = 5 * temp_diff * temp_diff / 2;
        SENS2         = 5 * temp_diff * temp_diff / 4;

        if (TEMP < -1500) {
            // Very low temperature (< -15°C)
            s64 temp_diff2 = static_cast<s64>(TEMP + 1500);
            OFF2 += 7 * temp_diff2 * temp_diff2;
            SENS2 += 11 * temp_diff2 * temp_diff2 / 2;
        }
    }

    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;

    // Calculate compensated pressure (0.01 mbar resolution)
    s32 P = static_cast<s32>(((static_cast<s64>(D1) * SENS >> 21) - OFF) >> 15);

    temperature = static_cast<float>(TEMP) / 100.f; // Convert to °C
    pressure    = static_cast<float>(P) / 100.f;    // Convert to mbar
}

LOGGER::STATUS MS5611::init()
{
    HAL_Delay(1);

    // Reset and reload PROM
    if (_soft_reset() != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;

    // Read factory calibration coefficients
    if (_read_calibration_data() != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;

    m_osr           = OSR_2048;
    m_conv_delay_ms = _conversion_delay_ms(m_osr);

    // Kick off the first D1 conversion so _read_both() can read it on its first call
    if (_send_command(_conv_d1_cmd(m_osr)) != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;
    m_us_time_last_sample = Timer::get_micro();
    m_state = SampleState::PRESSURE;

    return LOGGER::STATUS::OK;
}

LOGGER::STATUS MS5611::_read_both()
{
    u32 elapsed = Timer::get_micro() - m_us_time_last_sample;
    u32 wait_delay = 1000u * _conversion_delay_ms(m_osr);
    if (elapsed < wait_delay) return LOGGER::STATUS::OK;

    switch (m_state) {
    case SampleState::PRESSURE:
        // Read D1 result, then start D2 conversion
        if (_read_adc(m_D1) != LOGGER::STATUS::OK) {
            m_sensor_data.valid = false;
            return LOGGER::STATUS::LOGGER_ERROR;
        }
        if (_send_command(_conv_d2_cmd(m_osr)) != LOGGER::STATUS::OK) {
            m_sensor_data.valid = false;
            return LOGGER::STATUS::LOGGER_ERROR;
        }
        m_us_time_last_sample = Timer::get_micro();
        m_state = SampleState::TEMPERATURE;
        break;

    case SampleState::TEMPERATURE: {
        // Read D2 result, compensate, then start D1 conversion
        if (_read_adc(m_D2) != LOGGER::STATUS::OK) {
            m_sensor_data.valid = false;
            return LOGGER::STATUS::LOGGER_ERROR;
        }
        if (_send_command(_conv_d1_cmd(m_osr)) != LOGGER::STATUS::OK) {
            m_sensor_data.valid = false;
            return LOGGER::STATUS::LOGGER_ERROR;
        }
        m_us_time_last_sample = Timer::get_micro();

        float temperature = 0.f;
        float pressure    = 0.f;
        _compensate(m_D1, m_D2, temperature, pressure);
        m_sensor_data = {true, temperature, pressure, tx_time_get()};

        m_state = SampleState::PRESSURE;
        break;
    }
    }

    return LOGGER::STATUS::OK;
}

LOGGER::STATUS MS5611::read(Ms5611Data& data)
{
    LOGGER::STATUS status = _read_both();
    data                  = m_sensor_data;
    return status;
}

LOGGER::STATUS MS5611::sample_data()
{
    LOGGER::STATUS status = _read_both();
    return status;
}

float MS5611::get_last_temperature() { return m_sensor_data.temperature; }
float MS5611::get_last_pressure() { return m_sensor_data.pressure; }

