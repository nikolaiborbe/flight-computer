/**
 * @file h3lis331dl.hpp
 * @author Daniel Azagra Næss (daniel.ness@propulsentnu.no)
 * @brief Driver for the h3lis331dl accelerometer, based on our earlier c driver and rewritten to c++
 *
 * SOURCE: https://www.st.com/resource/en/datasheet/h3lis331dl.pdf
 *
 * @version 0.1
 * @date 2025-10-29
 *
 *
 */

#ifndef H3LIS331DL_H
#define H3LIS331DL_H

#include "IMU.h"
#include "logger.h"
#include "main.h"
#include "borbes_shorthands.h"

#ifdef __cplusplus
#include <cstdint>

// struct GPIO_TypeDef;
// struct SPI_HandleTypeDef;

// #include "stm32h7xx_hal_def.h"
// #include "stm32h7xx_hal_gpio.h"
// #include "stm32h7xx_hal_spi.h"
// #include "tx_api.h"

namespace H3lis331dl {
static constexpr uint8_t id = 0x32; // WhoAmI value, (dec)50, 0b00110010
struct SpiConfig {
    // Chip select
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;

    // SPI handle
    SPI_HandleTypeDef* hspi;
};

struct RawData {
    int16_t x;
    int16_t y;
    int16_t z;
};

struct Registers {
    static constexpr uint8_t who_am_i  = 0x0FU;
    static constexpr uint8_t ctrl_reg1 = 0x20U;
    static constexpr uint8_t ctrl_reg2 = 0x21U;
    static constexpr uint8_t ctrl_reg3 = 0x22U;
    static constexpr uint8_t ctrl_reg4 = 0x23U;
    static constexpr uint8_t ctrl_reg5 = 0x24U;
    static constexpr uint8_t hp_filter_reset
        = 0x25U; // Highpass filter reset. Attempting to read from this register will reset the filter
    static constexpr uint8_t hp_reference = 0x26U; // Reference value for highpass filter
    static constexpr uint8_t status_reg   = 0x27U; // Information about whether data is ready
    static constexpr uint8_t out_x_l      = 0x28U;
    static constexpr uint8_t out_x_h      = 0x29U;
    static constexpr uint8_t out_y_l      = 0x2AU;
    static constexpr uint8_t out_y_h      = 0x2BU;
    static constexpr uint8_t out_z_l      = 0x2CU;
    static constexpr uint8_t out_z_h      = 0x2DU;
};

struct InterruptRegisters {
    static constexpr uint8_t int1_cfg      = 0x30U;
    static constexpr uint8_t int1_src      = 0x31U;
    static constexpr uint8_t int1_ths      = 0x32U;
    static constexpr uint8_t int1_duration = 0x33U;
    static constexpr uint8_t int2_cfg      = 0x34U;
    static constexpr uint8_t int2_src      = 0x35U;
    static constexpr uint8_t int2_ths      = 0x36U;
    static constexpr uint8_t int2_duration = 0x37U;
};

enum class OUTPUT_DATA_RATE {
    ODR_0Hz5  = 0x40,
    ODR_OFF   = 0x00,
    ODR_1Hz   = 0x60,
    ODR_2Hz   = 0x80,
    ODR_5Hz   = 0xA0,
    ODR_10Hz  = 0xC0,
    ODR_50Hz  = 0x20,
    ODR_100Hz = 0x28,
    ODR_400Hz = 0x30,
    ODR_1kHz  = 0x38,
};

struct CtrlReg1 {
    static constexpr uint8_t addr       = Registers::ctrl_reg1;
    static constexpr uint8_t state_mask = 0xF8U;

    // Axis enables
    static constexpr uint8_t x_enable = 0x01U;
    static constexpr uint8_t y_enable = 0x02U;
    static constexpr uint8_t z_enable = 0x04U;

    static constexpr uint8_t axis_info_mask = 0x07U;
};

enum class AXIS_ENABLE : uint8_t {
    NONE = 0x00,
    X    = CtrlReg1::x_enable,
    Y    = CtrlReg1::y_enable,
    Z    = CtrlReg1::z_enable,
    XY   = (CtrlReg1::x_enable | CtrlReg1::y_enable),
    XZ   = (CtrlReg1::x_enable | CtrlReg1::z_enable),
    YZ   = (CtrlReg1::y_enable | CtrlReg1::z_enable),
    XYZ  = CtrlReg1::axis_info_mask
};

// CONTROL REGISTER 2
enum class HPF_MODE {
    HPF_NORMAL    = 0b00,
    HPF_REFERENCE = 0b01,
};

enum class HPF_CUTOFF_FREQUENCY {
    HPF_CUTOFF_COEFF_8  = 0b00,
    HPF_CUTOFF_COEFF_16 = 0b01,
    HPF_CUTOFF_COEFF_32 = 0b10,
    HPF_CUTOFF_COEFF_64 = 0b11,
};

enum class HPF_ENABLE {
    HP_DISABLE          = 0b000,
    HP_ON_OUT           = 0b100,
    HP_ON_INT1          = 0b001,
    HP_ON_INT2          = 0b010,
    HP_ON_INT1_INT2     = 0b011,
    HP_ON_INT1_INT2_OUT = 0b111,
    HP_ON_INT2_OUT      = 0b110,
    HP_ON_INT1_OUT      = 0b101,
};

struct CtrlReg2 {
    static constexpr uint8_t addr = Registers::ctrl_reg2;

    static constexpr uint8_t hpf_cutoff_frequency_mask = (0b11 << 0);
    static constexpr uint8_t hpf_cutoff_pos            = 0;

    static constexpr uint8_t hpf_int_enable_mask = (0b111 << 2);
    static constexpr uint8_t hpf_int_enable_pos  = 2;

    static constexpr uint8_t hpf_filtered_data_selection_mask = (1 << 4);
    static constexpr uint8_t hpf_fds_pos                      = 4;

    static constexpr uint8_t hpf_mode_mask = (0b11 << 5);
    static constexpr uint8_t hpf_mode_pos  = 5;

    static constexpr uint8_t boot_mask = (1 << 7);
    static constexpr uint8_t boot_pos  = 7;
};

// CONTROL REGISTER 3 (interrupts)
enum class INTERRUPT_CONFIG : uint8_t {
    SPECIFIC_SOURCE = 0b00, // Specific as in, int1 has int1 as source and int2 has int2 as source
    EITHER_SOURCE   = 0b01, // Either as in, int1 can be triggered by both int1 and int2
    DATA_READY      = 0b10,
    BOOT_RUNNING    = 0b11,
};
enum class INTERRUPT_LATCH : uint8_t {
    NOT_LATCHED = 0b0,
    LATCHED     = 0b1,
};
enum class INTERRUPT_PUSH_PULL_OPEN_DRAIN : uint8_t {
    PUSH_PULL  = 0b0,
    OPEN_DRAIN = 0b1,
};
enum class INTERRUPT_ACTIVE_LOW_HIGH : uint8_t {
    ACTIVE_HIGH = 0b0,
    ACTIVE_LOW  = 0b1,
};
struct CtrlReg3 {
    static constexpr uint8_t addr = Registers::ctrl_reg3;

    static constexpr uint8_t int1_cfg_mask = (0b11 << 0);
    static constexpr uint8_t int1_cfg_pos  = 0;

    static constexpr uint8_t latch_int1_request_mask = (0b1 << 2);
    static constexpr uint8_t latch_int1_pos          = 2;

    static constexpr uint8_t int2_cfg_mask = (0b11 << 3);
    static constexpr uint8_t int2_cfg_pos  = 3;

    static constexpr uint8_t latch_int2_request_mask = (0b1 << 5);
    static constexpr uint8_t latch_int2_request_pos  = 5;

    static constexpr uint8_t push_pull_open_drain_mask = (0b1 << 6);
    static constexpr uint8_t push_pull_open_drain_pos  = 6;

    static constexpr uint8_t int_active_low_high_mask = (0b1 << 7);
    static constexpr uint8_t int_active_low_high_pos  = 7;
};

// CONTROL REGISTER 4
enum class SPI_MODE : uint8_t {
    SPI_4_WIRE = 0b0,
    SPI_3_WIRE = 0b1,
};
enum class FULL_SCALE : uint8_t {
    FULL_SCALE_100g = 0b00,
    FULL_SCALE_200g = 0b01,
    FULL_SCALE_400g = 0b11,
};

enum class DATA_FORMAT : uint8_t {
    LITTLE_ENDIAN1 = 0b0, // TODO: check error here
    BIG_ENDIAN1    = 0b1,
};
enum class BLOCK_DATA_UPDATE : uint8_t {
    CONTINUOUS_UPDATE = 0b0,
    BLOCK_DATA_UPDATE = 0b1,
};
struct CtrlReg4 {
    static constexpr uint8_t addr = Registers::ctrl_reg4;

    static constexpr uint8_t spi_mode_mask = (0b1 << 0);
    static constexpr uint8_t spi_mode_pos  = 0;

    static constexpr uint8_t not_used = (0b111 << 1);

    static constexpr uint8_t full_scale_mask = (0b11 << 4);
    static constexpr uint8_t full_scale_pos  = 4;

    static constexpr uint8_t data_format_mask = (0b1 << 6);
    static constexpr uint8_t data_format_pos  = 6;

    static constexpr uint8_t block_data_update_mask = (0b1 << 7);
    static constexpr uint8_t block_data_update_pos  = 7;
};

// CONTROL REGISTER 5 (sleep-to-wake, switching between low-power and normal mode)
enum class SLEEP_TO_WAKE : uint8_t {
    SLEEP_TO_WAKE_DISABLED = 0b00,
    SLEEP_TO_WAKE_ENABLED  = 0b11,
};
struct CtrlReg5 {
    static constexpr uint8_t addr = Registers::ctrl_reg5;

    static constexpr uint8_t sleep_to_wake_mask = 0b11 << 0;
    static constexpr uint8_t sleep_to_wake_pos  = 0;

    static constexpr uint8_t reserved = static_cast<uint8_t>(~sleep_to_wake_mask);
};

// STATUS REGISTER
struct StatusReg {
    static constexpr uint8_t addr = Registers::status_reg;

    static constexpr uint8_t x_data_available   = 1 << 0;
    static constexpr uint8_t y_data_available   = 1 << 1;
    static constexpr uint8_t z_data_available   = 1 << 2;
    static constexpr uint8_t xyz_data_available = 1 << 3;
    static constexpr uint8_t x_data_overrun     = 1 << 4;
    static constexpr uint8_t y_data_overrun     = 1 << 5;
    static constexpr uint8_t z_data_overrun     = 1 << 6;
    static constexpr uint8_t xyz_data_overrun   = 1 << 7;
};

// INTERRUPT CONFIGURATION (same for 1 and 2)
enum class INTERRUPT_COMBINATION : uint8_t {
    INT_COMBINATION_OR  = 0b0,
    INT_COMBINATION_AND = 0b1,
};
struct IntCfg {
    static constexpr uint8_t x_low_interrupt_enable      = 1 << 0;
    static constexpr uint8_t x_high_interrupt_enable     = 1 << 1;
    static constexpr uint8_t y_low_interrupt_enable      = 1 << 2;
    static constexpr uint8_t y_high_interrupt_enable     = 1 << 3;
    static constexpr uint8_t z_low_interrupt_enable      = 1 << 4;
    static constexpr uint8_t z_high_interrupt_enable     = 1 << 5;
    static constexpr uint8_t not_used                    = 1 << 6;
    static constexpr uint8_t interrupt_event_combination = 1 << 7;
};

// INTERRUPT SOURCE (read-only, reading this clears the interrupt-active bit (and the interrupt signal on the int pin)
// and allows the refresh of data in the int_src register if the latched option is chosen)
struct IntSrc {
    static constexpr uint8_t x_low_interrupt_active  = 1 << 0;
    static constexpr uint8_t x_high_interrupt_active = 1 << 1;
    static constexpr uint8_t y_low_interrupt_active  = 1 << 2;
    static constexpr uint8_t y_high_interrupt_active = 1 << 3;
    static constexpr uint8_t z_low_interrupt_active  = 1 << 4;
    static constexpr uint8_t z_high_interrupt_active = 1 << 5;
    static constexpr uint8_t interrupt_active        = 1 << 6;
    static constexpr uint8_t not_used                = 1 << 7;
};

// INTERRUPT THRESHOLD
struct IntThreshold {
    uint8_t threshold;
};

// INTERRUPT DURATION
struct IntDuration {
    uint8_t duration;
};

// Configs
/**
 * @brief Struct for configuring high-pass-filter settings
 * 
 */
struct HpfConfig {
    HPF_MODE mode             = HPF_MODE::HPF_NORMAL;                       ///< The mode can be either normal or reference, see datasheet
    HPF_CUTOFF_FREQUENCY freq = HPF_CUTOFF_FREQUENCY::HPF_CUTOFF_COEFF_16;  ///< See page 19 of the datasheet
    HPF_ENABLE en_mask        = HPF_ENABLE::HP_ON_INT1;                     ///< See page 19 on the datasheet
};


/**
 * @brief Configuration struct for the sensor, used when calling the init() function.
 * 
 */
struct SensorConfig {
    OUTPUT_DATA_RATE odr  = OUTPUT_DATA_RATE::ODR_100Hz;            ///< The output data rate of the sensor
    FULL_SCALE fs         = FULL_SCALE::FULL_SCALE_100g;            ///< Full scale value can be either 100g, 200g or 400g
    BLOCK_DATA_UPDATE bdu = BLOCK_DATA_UPDATE::BLOCK_DATA_UPDATE;   ///< Block data update chooses if both the high and low registers for each axis will be updated continuously or if they only will update after they have both been read from.

    uint8_t axes_mask;

    HpfConfig hpf;
};
} // namespace H3lis331dl

/**
 * @brief Class for the h3lis331dl sensor, generate an instance of this class for your sensor
 */
class H3lis331dlDriver {
public:
        /**
     * @brief Construct a new H3lis331dlDriver object
     * 
     * @param io_config of type 
     */
    H3lis331dlDriver(const H3lis331dl::SpiConfig& io_config);

        /**
     * @brief Initializes the sensor
     * 
     * @return LOGGER::STATUS::OK if successful
     */
    LOGGER::STATUS init();

    /**
     * @brief Writes data to a register using spi
     * 
     * @param addr The register adress you want to write to 
     * @param data The data you want to write
     * @return LOGGER::STATUS::OK if the write was successful
     */
    LOGGER::STATUS spi_write(uint8_t addr, uint8_t data);

    /**
     * @brief Reads data from a register using spi
     * 
     * @param reg_addr The register you want to read.
     * @param[out] value Buffer where your data will get written
     * @return LOGGER::STATUS::OK if everything went fine
     */
    LOGGER::STATUS spi_read(uint8_t reg_addr, uint8_t& value);

    /**
     * @brief Reads more than one register at the time
     * 
     * @param reg_addr The start adress of the bytes you want to read
     * @param[out] buffer Where your data will be written
     * @param size How many bytes you want to read
     * @return LOGGER::STATUS::OK if successful
     */
    LOGGER::STATUS spi_read_multiple(uint8_t reg_addr, uint8_t* buffer, uint16_t size);

    /**
     * @brief Sets what axis should be measured
     * 
     * @param axes 
     * @return LOGGER::STATUS::OK if the function was successful
     */
    LOGGER::STATUS set_active_axis(H3lis331dl::AXIS_ENABLE axes);

    /**
     * @brief Sets the output data rate of the sensor
     * 
     * @param data_rate 
     * @return LOGGER::STATUS::OK if it was successfull
     */
    LOGGER::STATUS set_data_rate(H3lis331dl::OUTPUT_DATA_RATE data_rate);
    
    /**
     * @brief Sets the full scale configuration of the sensor. The sensor can have a max range of 100g, 200g or 400g.
     * 
     * @param fs
     * @return LOGGER::STATUS::OK if everything went ok.
     */
    LOGGER::STATUS set_full_scale(H3lis331dl::FULL_SCALE fs);
    
    /**
     * @brief Configures the high pass filter settings
     * 
     * @param mode Normal or reference
     * @param freq The frequency coefficient you want. Go to page 19 of the datasheet to see what each of them mean
     * @param en_mask What the hpf should be enabled for
     * @return LOGGER::STATUS::OK if succesful
     */
    LOGGER::STATUS configure_hpf(
        H3lis331dl::HPF_MODE mode, H3lis331dl::HPF_CUTOFF_FREQUENCY freq, H3lis331dl::HPF_ENABLE en_mask);

    /**
     * @brief Gets the acceleration in unit [g]
     * 
     * @param[out] accel output buffer
     * @return LOGGER::STATUS::OK if successful
     */
    LOGGER::STATUS get_acceleration(AccelerationData& accel);

    /**
     * @brief Reads the registers that contain the acceleration data and returns their values.
     * 
     * @param[out] raw buffer for reading the data. 
     * @return LOGGER::STATUS::OK if fine
     */
    LOGGER::STATUS get_raw_acceleration(H3lis331dl::RawData& raw);

    /**
     * @brief Set the sensitivity offset of the sensor.
     *
     * @note The sensor is sampling strange values; this is believed to be due to
     * the H3LIS sensor being a high-g sensor. To offset the measurements
     * I have followed to 3.1 Sensitivity section to find the sensitivity
     * offset the sensor is experiencing. This probably needs to be configured
     * over time, but right now the sensitivity offset seems quite stable.
     */
    void set_acceleration_sensitivity_offset(const AccelerationOffset& offset);

    /**
     * @brief Returns the sensitivity offset of the sensor.
     */
    AccelerationOffset get_acceleration_sensitivity_offset();

    // Helper functions

    /**
     * @brief Returns the correct conversion sensitivity based on what the current full_scale value is
     * 
     * @return float The sensitivity
     */
    float get_sensitivity() const;

    /**
     * @brief Returns if there is new data ready to read from the sensor.
     * 
     * @return true if there is new data
     * @return false 
     */
    bool is_data_ready();

private:
    H3lis331dl::SpiConfig _io;
    AccelerationOffset m_acceleration_offset {};

    // Caches
    uint8_t m_ctrl_reg1 { 0x00 };
    uint8_t m_ctrl_reg2 { 0x00 };
    uint8_t m_ctrl_reg3 { 0x00 };
    uint8_t m_ctrl_reg4 { 0x00 };
};

#endif

#endif
