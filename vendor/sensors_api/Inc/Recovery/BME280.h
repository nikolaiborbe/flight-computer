/**
 * @file BME280.h
 *
 * @author Nikolai G. Borbe (nikolai.borbe@propulsentnu.com)
 * @brief The header file for the BME280 (pressure, temperature, humidity) driver.
 * @version 1.0
 *
 * @date 2026-03-27
 */

#ifndef BME280_H
#define BME280_H

/**
 * GUIDE:
 * The sensor is NOT automatically initialized when you create an object. This means you need to
 * call init() before using the BME280 object.
 *
 * It is recommended to use the read(Bme280Data &data) function. Both for safety, and for speed. If
 * the user wants to sample a specific type of data they can use the read_<type>() functions. There
 * is also implementation to sample data from the entire register and only read the previous sampled
 * data. You do this by first calling the sample_data() function and then the get_last_<type>().
 *
 * NOTE: The BME280's specified pressure operating range is 300-1100 hPa. At ~10 km altitude,
 * pressure drops to ~264 hPa which is slightly below the rated range. The sensor will still output
 * data but accuracy is not guaranteed below 300 hPa. Default init() uses x16 pressure oversampling
 * with light IIR filtering (coeff 2) for minimal lag during fast ascent.
 */

#include "logger.h"
#include "main.h"
#include "tx_api.h"
#include "borbes_shorthands.h"

#ifdef __cplusplus
#include <cstdint>
#endif


constexpr uint32_t BME280_SPI_TIMEOUT_MS = 100;

struct BmeSpiConfig {
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;
    SPI_HandleTypeDef* hspi;
};

struct PressureData {
    bool valid;
    float pressure; // Pa
};

struct HumidityData {
    bool valid;
    float humidity; // %RH
};

struct TemperatureData;

struct Bme280Data {
    bool valid;
    float temperature; // Celsius
    float pressure;    // Pa
    float humidity;    // %RH
    ULONG timestamp;
};

struct Bme280Config {
    u8 osrs_t; // Temperature oversampling
    u8 osrs_p; // Pressure oversampling
    u8 osrs_h; // Humidity oversampling
    u8 filter; // IIR filter coefficient
    u8 t_sb;   // Standby time
    u8 mode;   // Operating mode
};

/**
 * @brief Calibration data read from the BME280's NVM.
 *
 * @note These are factory-programmed and unique per device.
 */
struct Bme280CalibData {
    u16 dig_T1;
    s16 dig_T2;
    s16 dig_T3;
    u16 dig_P1;
    s16 dig_P2;
    s16 dig_P3;
    s16 dig_P4;
    s16 dig_P5;
    s16 dig_P6;
    s16 dig_P7;
    s16 dig_P8;
    s16 dig_P9;
    u8 dig_H1;
    s16 dig_H2;
    u8 dig_H3;
    s16 dig_H4;
    s16 dig_H5;
    s8 dig_H6;
};

#ifdef __cplusplus
class BME280 {
public:
    BME280(const BmeSpiConfig& spi)
        : m_spi(spi) { };

private:
    Bme280Data m_sensor_data {};

private:
    /**
     * @brief Relevant data registers.
     */
    static constexpr u8 PRESS_MSB  = 0xF7;
    static constexpr u8 PRESS_LSB  = 0xF8;
    static constexpr u8 PRESS_XLSB = 0xF9;
    static constexpr u8 TEMP_MSB   = 0xFA;
    static constexpr u8 TEMP_LSB   = 0xFB;
    static constexpr u8 TEMP_XLSB  = 0xFC;
    static constexpr u8 HUM_MSB    = 0xFD;
    static constexpr u8 HUM_LSB    = 0xFE;

    /**
     * @brief Configuration and control registers.
     */
    static constexpr u8 CHIP_ID_REG = 0xD0;
    static constexpr u8 RESET_REG   = 0xE0;
    static constexpr u8 CTRL_HUM    = 0xF2;
    static constexpr u8 STATUS_REG  = 0xF3;
    static constexpr u8 CTRL_MEAS   = 0xF4;
    static constexpr u8 CONFIG_REG  = 0xF5;

    /**
     * @brief Calibration data registers.
     */
    static constexpr u8 CALIB00 = 0x88; // T and P calibration (0x88 - 0x9F, 26 bytes)
    static constexpr u8 CALIB26 = 0xE1; // H calibration (0xE1 - 0xE7, 7 bytes)

    /**
     * @brief Temperature oversampling options.
     */
    enum OSRS_T {
        OSRS_T_SKIP = 0x00,
        OSRS_T_x1   = 0x01,
        OSRS_T_x2   = 0x02,
        OSRS_T_x4   = 0x03,
        OSRS_T_x8   = 0x04,
        OSRS_T_x16  = 0x05,
    };

    /**
     * @brief Pressure oversampling options.
     */
    enum OSRS_P {
        OSRS_P_SKIP = 0x00,
        OSRS_P_x1   = 0x01,
        OSRS_P_x2   = 0x02,
        OSRS_P_x4   = 0x03,
        OSRS_P_x8   = 0x04,
        OSRS_P_x16  = 0x05,
    };

    /**
     * @brief Humidity oversampling options.
     */
    enum OSRS_H {
        OSRS_H_SKIP = 0x00,
        OSRS_H_x1   = 0x01,
        OSRS_H_x2   = 0x02,
        OSRS_H_x4   = 0x03,
        OSRS_H_x8   = 0x04,
        OSRS_H_x16  = 0x05,
    };

    /**
     * @brief IIR filter coefficient options.
     */
    enum FILTER {
        FILTER_OFF = 0x00,
        FILTER_2   = 0x01,
        FILTER_4   = 0x02,
        FILTER_8   = 0x03,
        FILTER_16  = 0x04,
    };

    /**
     * @brief Standby time options (normal mode cycle time).
     */
    enum T_SB {
        T_SB_0_5ms  = 0x00,
        T_SB_62_5ms = 0x01,
        T_SB_125ms  = 0x02,
        T_SB_250ms  = 0x03,
        T_SB_500ms  = 0x04,
        T_SB_1000ms = 0x05,
        T_SB_10ms   = 0x06,
        T_SB_20ms   = 0x07,
    };

    /**
     * @brief Operating mode options.
     */
    enum MODE {
        MODE_SLEEP  = 0x00,
        MODE_FORCED = 0x01,
        MODE_NORMAL = 0x03,
    };

    static constexpr u8 CHIP_ID_RESPONSE = 0x60;
    static constexpr u8 RESET_VALUE      = 0xB6;

public:
    /**
     * @brief Init function to setup configuration for the sensor.
     *
     * @return The status.
     */
    LOGGER::STATUS init();

    /**
     * @brief Reads the value of the BME280 sensor.
     *
     * @param data& Location to fill with data.
     *
     * @return The status.
     */
    LOGGER::STATUS read(Bme280Data& data);

    /**
     * @brief Read temperature data directly over SPI.
     *
     * @return Temperature data in Celsius.
     */
    float read_temperature();

    /**
     * @brief Read pressure data directly over SPI.
     *
     * @return Pressure data in Pa.
     */
    float read_pressure();

    /**
     * @brief Read humidity data directly over SPI.
     *
     * @return Humidity data in %RH.
     */
    float read_humidity();

    /**
     * @brief Reads entire relevant memory register at once.
     *
     * @note The idea with this function is to read the entire relevant register with one SPI
     * transfer instead of multiple.
     */
    LOGGER::STATUS sample_data();

    /**
     * @brief Get the last temperature value sampled by calling sample_data().
     *
     * @return Temperature in Celsius.
     */
    float get_last_temperature();

    /**
     * @brief Get the last pressure value sampled by calling sample_data().
     *
     * @return Pressure in Pa.
     */
    float get_last_pressure();

    /**
     * @brief Get the last humidity value sampled by calling sample_data().
     *
     * @return Humidity in %RH.
     */
    float get_last_humidity();

private:
    /**
     * @brief Read factory calibration data from the sensor.
     *
     * @return The status.
     */
    LOGGER::STATUS _read_calibration_data();

    /**
     * @brief Apply Bosch compensation formula for temperature.
     *
     * @param adc_T Raw 20-bit ADC temperature value.
     *
     * @return Compensated temperature in Celsius.
     *
     * @note Also updates m_t_fine which is needed for pressure and humidity compensation.
     */
    float _compensate_temperature(s32 adc_T);

    /**
     * @brief Apply Bosch compensation formula for pressure.
     *
     * @param adc_P Raw 20-bit ADC pressure value.
     *
     * @return Compensated pressure in Pa.
     *
     * @note Temperature must be compensated first (m_t_fine must be valid).
     */
    float _compensate_pressure(s32 adc_P);

    /**
     * @brief Apply Bosch compensation formula for humidity.
     *
     * @param adc_H Raw 16-bit ADC humidity value.
     *
     * @return Compensated humidity in %RH.
     *
     * @note Temperature must be compensated first (m_t_fine must be valid).
     */
    float _compensate_humidity(s32 adc_H);

    /**
     * @brief Helper function to read all raw data registers in one burst.
     */
    LOGGER::STATUS _spi_receive_data_from_register();

    /**
     * @brief Get the chip id from the CHIP_ID register.
     */
    LOGGER::STATUS _get_chip_id(u8& id);

    /**
     * @brief Soft resets the BME280.
     */
    LOGGER::STATUS _soft_reset();

    /**
     * @brief Helper to set chip select low.
     */
    void _cs_low();

    /**
     * @brief Helper to set chip select high.
     */
    void _cs_high();

    /**
     * @brief Transmit data over SPI.
     *
     * @param data Pointer to data buffer to transmit.
     * @param len Number of bytes to transmit.
     *
     * @return The status.
     */
    LOGGER::STATUS _spi_transmit(u8* data, u16 len);

    /**
     * @brief Receive data from a register over SPI.
     *
     * @param reg Register address to read from (read bit will be set automatically).
     * @param data Pointer to buffer to store received data.
     * @param len Number of bytes to receive.
     *
     * @return The status.
     */
    LOGGER::STATUS _spi_receive(u8 reg, u8* data, u16 len);

    /**
     * @brief Write a single byte to a register.
     */
    LOGGER::STATUS _write_reg(u8 reg, u8 value);

private:
    BmeSpiConfig m_spi;
    Bme280CalibData m_calib {};
    Bme280Config m_cfg {};
    float m_t_fine {};
};

#endif

#endif