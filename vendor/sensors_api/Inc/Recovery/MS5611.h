/**
 * @file MS5611.h
 *
 * @author Nikolai G. Borbe (nikolai.borbe@propulsentnu.com)
 * @brief The header file for the MS5611 (barometric pressure) driver.
 * @version 1.0
 *
 * @date 2026-03-27
 */
#ifndef MS5611_H
#define MS5611_H

/**
 * GUIDE:
 * The sensor is NOT automatically initialized when you create an object. This means you need to
 * call init() before using the MS5611 object.
 *
 * It is recommended to use the read(Ms5611Data &data) function. Both for safety, and for speed. If
 * the user wants to sample a specific type of data they can use the read_<type>() functions. There
 * is also implementation to sample data and only read the previous sampled data. You do this by
 * first calling the sample_data() function and then the get_last_<type>().
 *
 * NOTE: The MS5611 uses a command-based SPI protocol, not register-based. Each measurement
 * requires triggering a conversion (D1 for pressure, D2 for temperature), waiting for the ADC,
 * and then reading the result. A full read cycle at OSR 4096 takes ~20 ms.
 *
 * The MS5611's operating range is 10-1200 mbar (extended) with full accuracy at 450-1100 mbar.
 * At ~10 km altitude (~264 mbar) the sensor is within the extended linear range, making it
 * well-suited for high-altitude rocketry. Default init() uses OSR 4096 for best resolution
 * (0.012 mbar) with second order temperature compensation enabled.
 */

#include "logger.h"
#include "main.h"
#include "tx_api.h"
#include "borbes_shorthands.h"

#ifdef __cplusplus
#include <cstdint>
#endif

constexpr uint32_t MS5611_SPI_TIMEOUT_MS = 100;

struct Ms5611SpiConfig {
    GPIO_TypeDef* cs_port;
    u16 cs_pin;
    SPI_HandleTypeDef* hspi;
};

struct Ms5611Data {
    bool valid;
    float temperature; // Celsius
    float pressure;    // mbar
    ULONG timestamp;
};

enum class SampleState {
    PRESSURE,
    TEMPERATURE,
};

/**
 * @brief Calibration coefficients read from the MS5611's 128-bit PROM.
 *
 * @note These are factory-programmed and unique per device. Addresses 1-6 of the PROM.
 */
struct Ms5611CalibData {
    u16 C1; // Pressure sensitivity | SENS_T1
    u16 C2; // Pressure offset | OFF_T1
    u16 C3; // Temperature coefficient of pressure sensitivity | TCS
    u16 C4; // Temperature coefficient of pressure offset | TCO
    u16 C5; // Reference temperature | T_REF
    u16 C6; // Temperature coefficient of the temperature | TEMPSENS
};

#ifdef __cplusplus
class MS5611 {
public:
    MS5611(const Ms5611SpiConfig& spi)
        : m_spi(spi) { };

private:
    Ms5611Data m_sensor_data {};

private:
    /**
     * @brief SPI commands (datasheet page 9, Figure 4).
     */
    static constexpr u8 CMD_RESET    = 0x1E;
    static constexpr u8 CMD_ADC_READ = 0x00;
    static constexpr u8 CMD_PROM_READ = 0xA0; // base address, add (addr << 1)

    /**
     * @brief Convert D1 (pressure) commands per OSR.
     */
    static constexpr u8 CMD_CONV_D1_256  = 0x40;
    static constexpr u8 CMD_CONV_D1_512  = 0x42;
    static constexpr u8 CMD_CONV_D1_1024 = 0x44;
    static constexpr u8 CMD_CONV_D1_2048 = 0x46;
    static constexpr u8 CMD_CONV_D1_4096 = 0x48;

    /**
     * @brief Convert D2 (temperature) commands per OSR.
     */
    static constexpr u8 CMD_CONV_D2_256  = 0x50;
    static constexpr u8 CMD_CONV_D2_512  = 0x52;
    static constexpr u8 CMD_CONV_D2_1024 = 0x54;
    static constexpr u8 CMD_CONV_D2_2048 = 0x56;
    static constexpr u8 CMD_CONV_D2_4096 = 0x58;

    /**
     * @brief Oversampling ratio options.
     *
     * @note Higher OSR = better resolution but longer conversion time.
     * Conversion times (max): 256=0.60ms, 512=1.17ms, 1024=2.28ms, 2048=4.54ms, 4096=9.04ms.
     */
    enum OSR {
        OSR_256  = 0,
        OSR_512  = 1,
        OSR_1024 = 2,
        OSR_2048 = 3,
        OSR_4096 = 4,
    };

public:
    /**
     * @brief Init function to setup the sensor.
     *
     * @return The status.
     */
    LOGGER::STATUS init();

    /**
     * @brief Reads the value of the MS5611 sensor.
     *
     * @param data& Location to fill with data.
     *
     * @note This triggers both D1 (pressure) and D2 (temperature) conversions, waits for each,
     * and applies compensation with second order temperature correction.
     *
     * @return The status.
     */
    LOGGER::STATUS read(Ms5611Data& data);

    /**
     * @brief Read temperature data.
     *
     * @note Triggers a D2 conversion, waits, reads, and compensates.
     *
     * @return Temperature in Celsius.
     */
    float read_temperature();

    /**
     * @brief Read pressure data.
     *
     * @note Triggers both D2 (temperature) and D1 (pressure) conversions since temperature
     * is needed for pressure compensation.
     *
     * @return Pressure in mbar.
     */
    float read_pressure();

    /**
     * @brief Triggers conversions for both pressure and temperature and stores the result.
     *
     * @return The status.
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
     * @return Pressure in mbar.
     */
    float get_last_pressure();

private:
    /**
     * @brief Read factory calibration coefficients C1-C6 from PROM.
     *
     * @return The status.
     */
    LOGGER::STATUS _read_calibration_data();

    /**
     * @brief Trigger a conversion and read the 24-bit ADC result.
     *
     * @param conv_cmd The conversion command (D1 or D2 at chosen OSR).
     * @param result Output: the 24-bit ADC value.
     *
     * @return The status.
     */
    LOGGER::STATUS _convert_and_read(u8 conv_cmd, u32& result);

    /**
     * @brief Apply first and second order compensation (datasheet pages 7-8).
     *
     * @param D1 Raw 24-bit pressure ADC value.
     * @param D2 Raw 24-bit temperature ADC value.
     * @param temperature Output: compensated temperature in Celsius.
     * @param pressure Output: compensated pressure in mbar.
     */
    void _compensate(u32 D1, u32 D2, float& temperature, float& pressure);

    /**
     * @brief Full read cycle: convert D1 and D2, compensate, store in m_sensor_data.
     *
     * @return The status.
     */
    LOGGER::STATUS _read_both();

    /**
     * @brief Read a 16-bit word from a PROM address.
     *
     * @param addr PROM address (0-7).
     * @param result Output: the 16-bit PROM value.
     *
     * @return The status.
     */
    LOGGER::STATUS _read_prom(u8 addr, u16& result);

    /**
     * @brief Read the 24-bit ADC result after a conversion.
     *
     * @param result Output: the 24-bit ADC value.
     *
     * @return The status.
     */
    LOGGER::STATUS _read_adc(u32& result);

    /**
     * @brief Soft resets the MS5611.
     *
     * @note Reloads calibration PROM into internal registers. Takes ~2.8 ms.
     */
    LOGGER::STATUS _soft_reset();

    /**
     * @brief Get the conversion delay in ms for the selected OSR.
     *
     * @param osr The oversampling ratio.
     *
     * @return Delay in ms (ceiling of max conversion time).
     */
    u8 _conversion_delay_ms(OSR osr);

    /**
     * @brief Get the D1 conversion command for the selected OSR.
     */
    u8 _conv_d1_cmd(OSR osr);

    /**
     * @brief Get the D2 conversion command for the selected OSR.
     */
    u8 _conv_d2_cmd(OSR osr);

    /**
     * @brief Helper to set chip select low.
     */
    void _cs_low();

    /**
     * @brief Helper to set chip select high.
     */
    void _cs_high();

    /**
     * @brief Send a single command byte over SPI.
     *
     * @param cmd The command byte.
     *
     * @return The status.
     */
    LOGGER::STATUS _send_command(u8 cmd);

    /**
     * @brief Send a command and receive data over SPI.
     *
     * @param cmd Command byte to send.
     * @param data Pointer to buffer to store received data.
     * @param len Number of bytes to receive.
     *
     * @return The status.
     */
    LOGGER::STATUS _spi_cmd_receive(u8 cmd, u8* data, u16 len);

private:
    Ms5611SpiConfig m_spi;
    Ms5611CalibData m_calib {};
    OSR m_osr {};
    u8 m_conv_delay_ms {};
    u32 m_us_time_last_sample {};
    SampleState m_state = SampleState::PRESSURE;
    u32 m_D1 = 0;
    u32 m_D2 = 0;
};

#endif

#endif
