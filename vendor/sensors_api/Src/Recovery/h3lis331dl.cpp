/**
 * @file h3lis331dl.cpp
 *
 * @author Daniel Azagra Næss
 * @brief Implementation of the h3lis331dl (Accelerometer) driver
 *
 */
#include "h3lis331dl.h"
// #include "main.h"
// #include "stm32h7xx_hal.h"

H3lis331dlDriver::H3lis331dlDriver(const H3lis331dl::SpiConfig& io_config)
    : _io(io_config)
    , m_ctrl_reg1 { 0x00 }
    , m_ctrl_reg2 { 0x00 }
    , m_ctrl_reg3 { 0x00 }
    , m_ctrl_reg4 { 0x00 }
{

    if (_io.cs_port != nullptr) {
        HAL_GPIO_WritePin(_io.cs_port, _io.cs_pin, GPIO_PIN_SET);
    }
}

// Register Addresses [cite: 467]
#define H3LIS331DL_WHO_AM_I 0x0F
#define H3LIS331DL_CTRL_REG1 0x20
#define H3LIS331DL_CTRL_REG4 0x23

// Expected Device ID [cite: 482]
#define H3LIS331DL_ID 0x32

LOGGER::STATUS H3lis331dlDriver::init()
{
    uint8_t deviceId;

    // 1. Check Device ID
    // Reads the WHO_AM_I register to verify communication.
    spi_read(H3LIS331DL_WHO_AM_I, deviceId);
    if (deviceId != H3LIS331DL_ID) {
        return LOGGER::LOGGER_ERROR; // Hardware error or wrong device
    }

    // 2. Configure Power Mode and Data Rate (CTRL_REG1)
    // PM[2:0] = 001 (Normal Mode) [cite: 507]
    // DR[1:0] = 00  (50 Hz ODR)   [cite: 511]
    // Zen, Yen, Xen = 111 (Enable all axes) [cite: 493, 495, 497]
    // Value: 0010 0111 (0x27)
    spi_write(H3LIS331DL_CTRL_REG1, 0x27);

    // 3. Configure Full Scale and BDU (CTRL_REG4)
    // BDU = 1 (Block Data Update enabled - prevents reading torn data)
    // FS[1:0] = 00 (+/- 100g range)
    // SIM = 0 (4-wire SPI default/I2C) [cite: 580]
    // Value: 1000 0000 (0x80)
    spi_write(H3LIS331DL_CTRL_REG4, 0x80);

    return LOGGER::STATUS::OK;
}

// Should this function return AccelerationData
// or should it write to a out parameter and return LOGGER::STATUS as it does now
// Idk how much errorhandling people want but i think it's important to catch errors
// Also might want to add a status check, i have implemented the function is_data_ready()
// That could be used here if we want
LOGGER::STATUS H3lis331dlDriver::get_raw_acceleration(H3lis331dl::RawData& raw)
{
    uint8_t buffer[6] = { 0 };

    LOGGER::STATUS status = spi_read_multiple(H3lis331dl::Registers::out_x_l, buffer, 6);
    if (status == LOGGER::STATUS::OK) {
        raw.x = static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
        raw.y = static_cast<int16_t>((buffer[3] << 8) | buffer[2]);
        raw.z = static_cast<int16_t>((buffer[5] << 8) | buffer[4]);
    }
    return status;
}

LOGGER::STATUS H3lis331dlDriver::get_acceleration(AccelerationData& accel)
{
    H3lis331dl::RawData raw = { 0, 0, 0 };

    LOGGER::STATUS status = get_raw_acceleration(raw);
    if (status == LOGGER::STATUS::OK) {
        float sensitivity = get_sensitivity();
        AccelerationOffset offset = get_acceleration_sensitivity_offset();

        accel.acc_x = (static_cast<float>(raw.x) * sensitivity / 1000.0f) + offset.acc_offset_x;
        accel.acc_y = (static_cast<float>(raw.y) * sensitivity / 1000.0f) + offset.acc_offset_y;
        accel.acc_z = (static_cast<float>(raw.z) * sensitivity / 1000.0f) + offset.acc_offset_z;
    }

    return status;
}

LOGGER::STATUS H3lis331dlDriver::set_active_axis(H3lis331dl::AXIS_ENABLE axes)
{
    m_ctrl_reg1 &= ~H3lis331dl::CtrlReg1::axis_info_mask;

    m_ctrl_reg1 |= static_cast<uint8_t>(axes);
    LOGGER::STATUS status = spi_write(H3lis331dl::Registers::ctrl_reg1, m_ctrl_reg1);
    return status;
}

LOGGER::STATUS H3lis331dlDriver::set_data_rate(H3lis331dl::OUTPUT_DATA_RATE data_rate)
{
    m_ctrl_reg1 &= ~H3lis331dl::CtrlReg1::state_mask;
    m_ctrl_reg1 |= static_cast<uint8_t>(data_rate);

    return spi_write(H3lis331dl::Registers::ctrl_reg1, m_ctrl_reg1);
}

LOGGER::STATUS H3lis331dlDriver::set_full_scale(H3lis331dl::FULL_SCALE fs)
{
    m_ctrl_reg4 &= ~H3lis331dl::CtrlReg4::full_scale_mask;
    m_ctrl_reg4 |= static_cast<uint8_t>(fs) << H3lis331dl::CtrlReg4::full_scale_pos;

    return spi_write(H3lis331dl::Registers::ctrl_reg4, m_ctrl_reg4);
}

LOGGER::STATUS H3lis331dlDriver::configure_hpf(
    H3lis331dl::HPF_MODE mode, H3lis331dl::HPF_CUTOFF_FREQUENCY freq, H3lis331dl::HPF_ENABLE en_mask)
{
    const uint8_t hpf_bits_mask = H3lis331dl::CtrlReg2::hpf_mode_mask | H3lis331dl::CtrlReg2::hpf_cutoff_frequency_mask
        | (0x07 << H3lis331dl::CtrlReg2::hpf_int_enable_pos);

    m_ctrl_reg2 &= ~hpf_bits_mask;
    m_ctrl_reg2 |= static_cast<uint8_t>(mode) << H3lis331dl::CtrlReg2::hpf_mode_pos;
    m_ctrl_reg2 |= static_cast<uint8_t>(freq) << H3lis331dl::CtrlReg2::hpf_cutoff_pos;
    m_ctrl_reg2 |= static_cast<uint8_t>(en_mask) << H3lis331dl::CtrlReg2::hpf_int_enable_pos;

    return (spi_write(H3lis331dl::Registers::ctrl_reg2, m_ctrl_reg2));
}

float H3lis331dlDriver::get_sensitivity() const
{
    uint8_t fs = (m_ctrl_reg4 & H3lis331dl::CtrlReg4::full_scale_mask) >> H3lis331dl::CtrlReg4::full_scale_pos;

    switch (static_cast<H3lis331dl::FULL_SCALE>(fs)) {
    case H3lis331dl::FULL_SCALE::FULL_SCALE_100g: return 3.0625f;
    case H3lis331dl::FULL_SCALE::FULL_SCALE_200g: return 6.125f;
    case H3lis331dl::FULL_SCALE::FULL_SCALE_400g: return 12.1875f;
    default: return 3.0625f;
    }
}

LOGGER::STATUS H3lis331dlDriver::spi_write(uint8_t addr, uint8_t data)
{
    uint8_t tx_buf[2] = { static_cast<uint8_t>(addr & 0x3F), data };

    HAL_GPIO_WritePin(_io.cs_port, _io.cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(_io.hspi, tx_buf, 2, SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(_io.cs_port, _io.cs_pin, GPIO_PIN_SET);

    return (status == HAL_OK) ? LOGGER::STATUS::OK : LOGGER::STATUS::LOGGER_ERROR;
}

LOGGER::STATUS H3lis331dlDriver::spi_read(uint8_t reg_addr, uint8_t& value)
{
    uint8_t tx_data = reg_addr | 0x80;
    uint8_t rx_data = 0;

    HAL_GPIO_WritePin(_io.cs_port, _io.cs_pin, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(_io.hspi, &tx_data, 1, 10) != HAL_OK) {
        HAL_GPIO_WritePin(_io.cs_port, _io.cs_pin, GPIO_PIN_SET);
        return LOGGER::STATUS::LOGGER_ERROR;
    }

    if (HAL_SPI_Receive(_io.hspi, &rx_data, 1, 10) != HAL_OK) {
        HAL_GPIO_WritePin(_io.cs_port, _io.cs_pin, GPIO_PIN_SET);
        return LOGGER::STATUS::LOGGER_ERROR;
    }

    HAL_GPIO_WritePin(_io.cs_port, _io.cs_pin, GPIO_PIN_SET);

    value = rx_data;

    return LOGGER::STATUS::OK;
}

LOGGER::STATUS H3lis331dlDriver::spi_read_multiple(uint8_t reg_addr, uint8_t* buffer, uint16_t size)
{
    u8 tx_data = static_cast<u8>(reg_addr | 0x80 | 0x40);

    HAL_GPIO_WritePin(_io.cs_port, _io.cs_pin, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(_io.hspi, &tx_data, 1, 10) != HAL_OK) {
        HAL_GPIO_WritePin(_io.cs_port, _io.cs_pin, GPIO_PIN_SET);
        return LOGGER::STATUS::LOGGER_ERROR;
    }

    if (HAL_SPI_Receive(_io.hspi, buffer, size, 10) != HAL_OK) {
        HAL_GPIO_WritePin(_io.cs_port, _io.cs_pin, GPIO_PIN_SET);
        return LOGGER::STATUS::LOGGER_ERROR;
    }

    HAL_GPIO_WritePin(_io.cs_port, _io.cs_pin, GPIO_PIN_SET);
    return LOGGER::STATUS::OK;
}

bool H3lis331dlDriver::is_data_ready()
{
    uint8_t status = 0;
    if (spi_read(H3lis331dl::StatusReg::addr, status) != LOGGER::STATUS::OK) {
        return false;
    }
    return (status & H3lis331dl::StatusReg::xyz_data_available) != 0;
}

void H3lis331dlDriver::set_acceleration_sensitivity_offset(const AccelerationOffset& offset) {
    m_acceleration_offset.acc_offset_x = offset.acc_offset_x;
    m_acceleration_offset.acc_offset_y = offset.acc_offset_y;
    m_acceleration_offset.acc_offset_z = offset.acc_offset_z;
}

AccelerationOffset H3lis331dlDriver::get_acceleration_sensitivity_offset() { return m_acceleration_offset; };
