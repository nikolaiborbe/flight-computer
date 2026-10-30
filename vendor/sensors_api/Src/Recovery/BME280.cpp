/**
 * @file BME280.cpp
 *
 * @author Nikolai G. Borbe (nikolai.borbe@propulsentnu.com)
 * @brief This file contains the implementation for the BME280 (pressure, temperature, humidity)
 * driver.
 * @version 1.0
 *
 * @date 2026-03-27
 */

#include "BME280.h"

void BME280::_cs_low() { HAL_GPIO_WritePin(m_spi.cs_port, m_spi.cs_pin, GPIO_PIN_RESET); }
void BME280::_cs_high() { HAL_GPIO_WritePin(m_spi.cs_port, m_spi.cs_pin, GPIO_PIN_SET); }

LOGGER::STATUS BME280::_spi_transmit(u8* data, u16 len)
{
    _cs_low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(m_spi.hspi, data, len, BME280_SPI_TIMEOUT_MS);
    _cs_high();
    return (status == HAL_OK) ? LOGGER::STATUS::OK : LOGGER::STATUS::LOGGER_ERROR;
}

LOGGER::STATUS BME280::_spi_receive(u8 reg, u8* data, u16 len)
{
    u8 tx_reg = (reg & 0x7F) | 0x80;
    _cs_low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(m_spi.hspi, &tx_reg, 1, BME280_SPI_TIMEOUT_MS);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(m_spi.hspi, data, len, BME280_SPI_TIMEOUT_MS);
    }
    _cs_high();
    return (status == HAL_OK) ? LOGGER::STATUS::OK : LOGGER::STATUS::LOGGER_ERROR;
}

LOGGER::STATUS BME280::_write_reg(u8 reg, u8 value)
{
    u8 tx[2] = {static_cast<u8>(reg & 0x7F), value};
    return _spi_transmit(tx, 2);
}

LOGGER::STATUS BME280::_soft_reset()
{
    LOGGER::STATUS status = _write_reg(RESET_REG, RESET_VALUE);
    HAL_Delay(2); // startup time after reset (see datasheet section 1.1)
    return status;
}

LOGGER::STATUS BME280::_get_chip_id(u8& id)
{
    u8 rx                 = 0;
    LOGGER::STATUS status = _spi_receive(CHIP_ID_REG, &rx, 1);
    id                    = rx;
    return status;
}

LOGGER::STATUS BME280::_read_calibration_data()
{
    // Temperature and pressure calibration: registers 0x88 - 0xA1 (26 bytes)
    u8 tp[26] = {};
    if (_spi_receive(CALIB00, tp, 26) != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;

    m_calib.dig_T1 = static_cast<u16>(tp[1]) << 8 | tp[0];
    m_calib.dig_T2 = static_cast<s16>(static_cast<u16>(tp[3]) << 8 | tp[2]);
    m_calib.dig_T3 = static_cast<s16>(static_cast<u16>(tp[5]) << 8 | tp[4]);

    m_calib.dig_P1 = static_cast<u16>(tp[7]) << 8 | tp[6];
    m_calib.dig_P2 = static_cast<s16>(static_cast<u16>(tp[9]) << 8 | tp[8]);
    m_calib.dig_P3 = static_cast<s16>(static_cast<u16>(tp[11]) << 8 | tp[10]);
    m_calib.dig_P4 = static_cast<s16>(static_cast<u16>(tp[13]) << 8 | tp[12]);
    m_calib.dig_P5 = static_cast<s16>(static_cast<u16>(tp[15]) << 8 | tp[14]);
    m_calib.dig_P6 = static_cast<s16>(static_cast<u16>(tp[17]) << 8 | tp[16]);
    m_calib.dig_P7 = static_cast<s16>(static_cast<u16>(tp[19]) << 8 | tp[18]);
    m_calib.dig_P8 = static_cast<s16>(static_cast<u16>(tp[21]) << 8 | tp[20]);
    m_calib.dig_P9 = static_cast<s16>(static_cast<u16>(tp[23]) << 8 | tp[22]);

    // dig_H1 is at 0xA1 (tp[25])
    m_calib.dig_H1 = tp[25];

    // Humidity calibration: registers 0xE1 - 0xE7 (7 bytes)
    u8 h[7] = {};
    if (_spi_receive(CALIB26, h, 7) != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;

    m_calib.dig_H2 = static_cast<s16>(static_cast<u16>(h[1]) << 8 | h[0]);
    m_calib.dig_H3 = h[2];
    m_calib.dig_H4 = static_cast<s16>(static_cast<s16>(h[3]) << 4 | (h[4] & 0x0F));
    m_calib.dig_H5 = static_cast<s16>(static_cast<s16>(h[5]) << 4 | (h[4] >> 4));
    m_calib.dig_H6 = static_cast<s8>(h[6]);

    return LOGGER::STATUS::OK;
}

float BME280::_compensate_temperature(s32 adc_T)
{
    // Bosch compensation formula (datasheet section 4.2.3)
    float var1 = (static_cast<float>(adc_T) / 16384.f - static_cast<float>(m_calib.dig_T1) / 1024.f)
                 * static_cast<float>(m_calib.dig_T2);
    float diff = static_cast<float>(adc_T) / 131072.f - static_cast<float>(m_calib.dig_T1) / 8192.f;
    float var2 = diff * diff * static_cast<float>(m_calib.dig_T3);

    m_t_fine = var1 + var2;
    return m_t_fine / 5120.f;
}

float BME280::_compensate_pressure(s32 adc_P)
{
    // Bosch compensation formula (datasheet section 4.2.3)
    float var1 = m_t_fine / 2.f - 64000.f;
    float var2 = var1 * var1 * static_cast<float>(m_calib.dig_P6) / 32768.f;
    var2       = var2 + var1 * static_cast<float>(m_calib.dig_P5) * 2.f;
    var2       = var2 / 4.f + static_cast<float>(m_calib.dig_P4) * 65536.f;
    var1       = (static_cast<float>(m_calib.dig_P3) * var1 * var1 / 524288.f
                  + static_cast<float>(m_calib.dig_P2) * var1)
                 / 524288.f;
    var1       = (1.f + var1 / 32768.f) * static_cast<float>(m_calib.dig_P1);

    if (var1 == 0.f)
        return 0.f; // avoid division by zero

    float p = 1048576.f - static_cast<float>(adc_P);
    p       = (p - var2 / 4096.f) * 6250.f / var1;
    var1    = static_cast<float>(m_calib.dig_P9) * p * p / 2147483648.f;
    var2    = p * static_cast<float>(m_calib.dig_P8) / 32768.f;

    return p + (var1 + var2 + static_cast<float>(m_calib.dig_P7)) / 16.f;
}

float BME280::_compensate_humidity(s32 adc_H)
{
    // Bosch compensation formula (datasheet section 4.2.3)
    float h = m_t_fine - 76800.f;
    if (h == 0.f)
        return 0.f;

    h = (static_cast<float>(adc_H)
         - (static_cast<float>(m_calib.dig_H4) * 64.f
            + static_cast<float>(m_calib.dig_H5) / 16384.f * h))
        * (static_cast<float>(m_calib.dig_H2) / 65536.f
           * (1.f
              + static_cast<float>(m_calib.dig_H6) / 67108864.f * h
                    * (1.f + static_cast<float>(m_calib.dig_H3) / 67108864.f * h)));

    h = h * (1.f - static_cast<float>(m_calib.dig_H1) * h / 524288.f);

    if (h > 100.f)
        h = 100.f;
    if (h < 0.f)
        h = 0.f;

    return h;
}

LOGGER::STATUS BME280::init()
{
    HAL_Delay(2);

    // Reset
    if (_soft_reset() != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;

    // Verify chip identity
    u8 id {};
    if (_get_chip_id(id) != LOGGER::STATUS::OK || id != CHIP_ID_RESPONSE)
        return LOGGER::STATUS::LOGGER_ERROR;

    // Read factory calibration data
    if (_read_calibration_data() != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;

    // Configure for high-altitude rocketry (~10 km):
    // - x16 pressure oversampling for best altitude resolution
    // - x2 temperature oversampling (needed for compensation)
    // - x1 humidity oversampling (not critical for altitude)
    // - IIR filter coeff 2 (light filtering, minimal lag during fast ascent)
    // - 0.5ms standby (fastest cycle time)
    // - Normal mode (continuous measurement)
    m_cfg = {OSRS_T_x2, OSRS_P_x16, OSRS_H_x1, FILTER_2, T_SB_0_5ms, MODE_NORMAL};

    // ctrl_hum must be written before ctrl_meas for changes to take effect (datasheet
    // section 5.4.3)
    if (_write_reg(CTRL_HUM, m_cfg.osrs_h) != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;

    // config register: t_sb[7:5], filter[4:2], spi3w_en[0]
    u8 config_val = (m_cfg.t_sb << 5) | (m_cfg.filter << 2);
    if (_write_reg(CONFIG_REG, config_val) != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;

    // ctrl_meas: osrs_t[7:5], osrs_p[4:2], mode[1:0]
    u8 ctrl_meas_val = (m_cfg.osrs_t << 5) | (m_cfg.osrs_p << 2) | m_cfg.mode;
    if (_write_reg(CTRL_MEAS, ctrl_meas_val) != LOGGER::STATUS::OK)
        return LOGGER::STATUS::LOGGER_ERROR;

    return LOGGER::STATUS::OK;
}

float BME280::read_temperature()
{
    u8 rx[3] = {};
    if (_spi_receive(TEMP_MSB, rx, 3) != LOGGER::STATUS::OK)
        return 0.f;

    s32 adc_T = (static_cast<s32>(rx[0]) << 12) | (static_cast<s32>(rx[1]) << 4)
                | (static_cast<s32>(rx[2]) >> 4);
    return _compensate_temperature(adc_T);
}

float BME280::read_pressure()
{
    // Temperature must be read first to update m_t_fine
    read_temperature();

    u8 rx[3] = {};
    if (_spi_receive(PRESS_MSB, rx, 3) != LOGGER::STATUS::OK)
        return 0.f;

    s32 adc_P = (static_cast<s32>(rx[0]) << 12) | (static_cast<s32>(rx[1]) << 4)
                | (static_cast<s32>(rx[2]) >> 4);
    return _compensate_pressure(adc_P);
}

float BME280::read_humidity()
{
    // Temperature must be read first to update m_t_fine
    read_temperature();

    u8 rx[2] = {};
    if (_spi_receive(HUM_MSB, rx, 2) != LOGGER::STATUS::OK)
        return 0.f;

    s32 adc_H = (static_cast<s32>(rx[0]) << 8) | static_cast<s32>(rx[1]);
    return _compensate_humidity(adc_H);
}

float BME280::get_last_temperature() { return m_sensor_data.temperature; }
float BME280::get_last_pressure() { return m_sensor_data.pressure; }
float BME280::get_last_humidity() { return m_sensor_data.humidity; }

LOGGER::STATUS BME280::read(Bme280Data& data)
{
    LOGGER::STATUS status = _spi_receive_data_from_register();
    data                  = m_sensor_data;
    return status;
}

LOGGER::STATUS BME280::sample_data()
{
    LOGGER::STATUS status = _spi_receive_data_from_register();
    return status;
}

LOGGER::STATUS BME280::_spi_receive_data_from_register()
{
    // Burst read: press[3] + temp[3] + hum[2] = 8 bytes starting at 0xF7
    u8 rx[8] = {0};
    if (_spi_receive(PRESS_MSB, rx, 8) != LOGGER::STATUS::OK) {
        m_sensor_data.valid = false;
        return LOGGER::STATUS::LOGGER_ERROR;
    }

    // Assemble 20-bit pressure (MSB, LSB, XLSB[7:4])
    s32 adc_P = (static_cast<s32>(rx[0]) << 12) | (static_cast<s32>(rx[1]) << 4)
                | (static_cast<s32>(rx[2]) >> 4);

    // Assemble 20-bit temperature (MSB, LSB, XLSB[7:4])
    s32 adc_T = (static_cast<s32>(rx[3]) << 12) | (static_cast<s32>(rx[4]) << 4)
                | (static_cast<s32>(rx[5]) >> 4);

    // Assemble 16-bit humidity (MSB, LSB)
    s32 adc_H = (static_cast<s32>(rx[6]) << 8) | static_cast<s32>(rx[7]);

    // Temperature must be compensated first (updates m_t_fine used by pressure and humidity)
    float temperature = _compensate_temperature(adc_T);
    float pressure    = _compensate_pressure(adc_P);
    float humidity    = _compensate_humidity(adc_H);

    m_sensor_data = {true, temperature, pressure, humidity, tx_time_get()};

    return LOGGER::STATUS::OK;
}
