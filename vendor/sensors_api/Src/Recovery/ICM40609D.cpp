/**
 * @file ICM40609D.cpp
 *
 * @author Nikolai G. Borbe (nikolai.borbe@propulsentnu.com)
 * @brief This file contains the implementation for the ICM-40609-D (IMU) driver.
 * @version 1.0
 *
 * @date 2026-03-26
 */

#include "ICM40609D.h"

static constexpr s16 icm_be16_to_i16(u8 msb, u8 lsb)
{
    return static_cast<s16>((static_cast<u16>(msb) << 8) | static_cast<u16>(lsb));
}

static constexpr float icm_be16_to_float(u8 msb, u8 lsb) { return static_cast<float>(icm_be16_to_i16(msb, lsb)); }

static constexpr s16 icm_s12_to_i16(u16 raw)
{
    raw &= 0x0FFF;
    return static_cast<s16>((raw & 0x0800) ? (raw | 0xF000) : raw);
}

void ICM40609D::_cs_low() { HAL_GPIO_WritePin(m_spi.cs_port, m_spi.cs_pin, GPIO_PIN_RESET); }
void ICM40609D::_cs_high() { HAL_GPIO_WritePin(m_spi.cs_port, m_spi.cs_pin, GPIO_PIN_SET); }

int ICM40609D::_spi_transmit(u8* data, u16 len)
{
    _cs_low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(m_spi.hspi, data, len, SPI_TIMEOUT_MS);
    _cs_high();
    return (status == HAL_OK) ? OK : ERR_SPI;
}

int ICM40609D::_spi_receive(u8 reg, u8* data, u16 len)
{
    u8 tx_reg = (reg & 0x7F) | 0x80;
    _cs_low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(m_spi.hspi, &tx_reg, 1, SPI_TIMEOUT_MS);
    if (status == HAL_OK) {
        status = HAL_SPI_Receive(m_spi.hspi, data, len, SPI_TIMEOUT_MS);
    }
    _cs_high();
    return (status == HAL_OK) ? OK : ERR_SPI;
}

int ICM40609D::_write_reg(u8 reg, u8 value)
{
    u8 tx[2] = { static_cast<u8>(reg & 0x7F), value };
    return _spi_transmit(tx, 2);
}

float ICM40609D::_accel_sensitivity_scale_factor(ACCEL_FS_SEL full_scale_select)
{
    switch (full_scale_select) {
    case ACCEL_FS_SEL::ACCEL_FS_SEL_32g: return 1024.f;
    case ACCEL_FS_SEL::ACCEL_FS_SEL_16g: return 2048.f;
    case ACCEL_FS_SEL::ACCEL_FS_SEL_8g: return 4096.f;
    case ACCEL_FS_SEL::ACCEL_FS_SEL_4g: return 8192.f;
    default: return 1024.f;
    }
}

float ICM40609D::_gyro_sensitivity_scale_factor(GYRO_FS_SEL full_scale_select)
{
    switch (full_scale_select) {
    case GYRO_FS_SEL::GYRO_FS_SEL_2000: return 16.4f;
    case GYRO_FS_SEL::GYRO_FS_SEL_1000: return 32.8f;
    case GYRO_FS_SEL::GYRO_FS_SEL_500: return 65.5f;
    case GYRO_FS_SEL::GYRO_FS_SEL_250: return 131.f;
    case GYRO_FS_SEL::GYRO_FS_SEL_125: return 262.f;
    case GYRO_FS_SEL::GYRO_FS_SEL_63: return 524.3f;
    case GYRO_FS_SEL::GYRO_FS_SEL_31: return 1048.6f;
    case GYRO_FS_SEL::GYRO_FS_SEL_16: return 2097.2f;
    default: return 16.4f;
    }
}

int ICM40609D::_toggle_accel_gyro_temp(DEVICE_STATUS st)
{
    u8 val {};

    switch (st) {
    // PWR_MGMT0: bit5=TEMP_DIS, bit4=IDLE, bits3:2=GYRO_MODE, bits1:0=ACCEL_MODE
    // OFF: temp disabled, RC oscillator idle on, gyro off, accel off
    case DEVICE_STATUS::OFF: val = (1u << 5) | (1u << 4) | (0u << 2) | 0u; break;
    // ON: temp enabled, gyro LN mode (11), accel LN mode (11)
    case DEVICE_STATUS::ON: val = (0u << 5) | (3u << 2) | 3u; break;
    }

    u8 tx[2]              = { (PWR_MGMT0 & 0x7F), val };
    int status = _spi_transmit(tx, 2);
    HAL_Delay(45); // delay for gyroscope startup

    return status;
}

int ICM40609D::_soft_reset()
{
    u8 tx[2]              = { (DEVICE_CONFIG & 0x7F), 0x01 };
    int status = _spi_transmit(tx, 2);
    HAL_Delay(1); // required by sensor
    return status;
}

int ICM40609D::_set_accel_offsets(float offset_x_g, float offset_y_g, float offset_z_g)
{
    if (_set_offset_acc_x(offset_x_g) != OK)
        return ERR_SPI;
    if (_set_offset_acc_y(offset_y_g) != OK)
        return ERR_SPI;
    if (_set_offset_acc_z(offset_z_g) != OK)
        return ERR_SPI;

    return OK;
}

int ICM40609D::_set_offset_acc_x(float offset_g)
{
    constexpr float res_g = 0.0005f;
    constexpr s16 min     = -2000;
    constexpr s16 max     = 2000;

    _bank_select(BANK_SEL_4);

    u8 upper_gyro_z = _get_register_value(OFFSET_USER4);

    s16 steps        = std::clamp(static_cast<s16>(std::lround(offset_g / res_g)), min, max);
    u16 raw12        = static_cast<u16>(steps) & 0x0FFF;
    u8 lower         = static_cast<u8>(raw12 & 0x00FF);
    u8 upper         = static_cast<u8>((raw12 >> 8) & 0x0F);
    u8 proper_upper = static_cast<u8>((upper << 4) | (upper_gyro_z & 0x0F));

    int status = _write_reg(OFFSET_USER4, proper_upper);
    if (status == OK)
        status = _write_reg(OFFSET_USER5, lower);

    _bank_select(BANK_SEL_0);

    return status;
}

int ICM40609D::_set_offset_acc_y(float offset_g)
{
    constexpr float res_g = 0.0005f;
    constexpr s16 min     = -2000;
    constexpr s16 max     = 2000;

    _bank_select(BANK_SEL_4);

    u8 upper_accel_z = _get_register_value(OFFSET_USER7);

    s16 steps        = std::clamp(static_cast<s16>(std::lround(offset_g / res_g)), min, max);
    u16 raw12        = static_cast<u16>(steps) & 0x0FFF;
    u8 lower         = static_cast<u8>(raw12 & 0x00FF);
    u8 upper         = static_cast<u8>((raw12 >> 8) & 0x0F);
    u8 proper_upper = static_cast<u8>((upper_accel_z & 0xF0) | upper);

    int status = _write_reg(OFFSET_USER6, lower);
    if (status == OK)
        status = _write_reg(OFFSET_USER7, proper_upper);

    _bank_select(BANK_SEL_0);

    return status;
}

int ICM40609D::_set_offset_acc_z(float offset_g)
{
    constexpr float res_g = 0.0005f;
    constexpr s16 min     = -2000;
    constexpr s16 max     = 2000;

    _bank_select(BANK_SEL_4);

    u8 upper_accel_y = _get_register_value(OFFSET_USER7);

    s16 steps        = std::clamp(static_cast<s16>(std::lround(offset_g / res_g)), min, max);
    u16 raw12        = static_cast<u16>(steps) & 0x0FFF;
    u8 lower         = static_cast<u8>(raw12 & 0x00FF);
    u8 upper         = static_cast<u8>((raw12 >> 8) & 0x0F);
    u8 proper_upper = static_cast<u8>((upper << 4) | (upper_accel_y & 0x0F));

    int status = _write_reg(OFFSET_USER7, proper_upper);
    if (status == OK)
        status = _write_reg(OFFSET_USER8, lower);

    _bank_select(BANK_SEL_0);

    return status;
}

AccelerationOffset ICM40609D::read_accel_offset_registers()
{
    constexpr float res_g = 0.0005f;

    _bank_select(BANK_SEL_4);

    const u8 offset_user4 = _get_register_value(OFFSET_USER4);
    const u8 offset_user5 = _get_register_value(OFFSET_USER5);
    const u8 offset_user6 = _get_register_value(OFFSET_USER6);
    const u8 offset_user7 = _get_register_value(OFFSET_USER7);
    const u8 offset_user8 = _get_register_value(OFFSET_USER8);

    _bank_select(BANK_SEL_0);

    const u16 raw_x = (static_cast<u16>(offset_user4 >> 4) << 8) | offset_user5;
    const u16 raw_y = (static_cast<u16>(offset_user7 & 0x0F) << 8) | offset_user6;
    const u16 raw_z = (static_cast<u16>(offset_user7 >> 4) << 8) | offset_user8;

    return AccelerationOffset {
        static_cast<float>(icm_s12_to_i16(raw_x)) * res_g,
        static_cast<float>(icm_s12_to_i16(raw_y)) * res_g,
        static_cast<float>(icm_s12_to_i16(raw_z)) * res_g,
    };
}

int ICM40609D::init(float acc_offset_x, float acc_offset_y, float acc_offset_z)
{
    m_initialized = false;
    m_sensor_data.valid = false;
    m_accel_offset_g = { acc_offset_x, acc_offset_y, acc_offset_z };

    HAL_Delay(1);

    // Reset and power down for configuration
    if (_soft_reset() != OK)
        return ERR_SPI;
    if (_toggle_accel_gyro_temp(DEVICE_STATUS::OFF) != OK)
        return ERR_SPI;

    // Verify device identity
    u8 id {};
    int ret = _get_device_id(id);
    if (ret != OK)
        return ret;
    if (id != WHO_AM_I_RESPONSE_ID)
        return ERR_INVALID_DEVICE;

    m_accel_cfg
        = { ACCEL_FS_SEL_16g, ACCEL_ODR_200Hz, ACCEL_UI_FILT_BW_4, ACCEL_UI_FILT_ORD_3rd, ACCEL_DEC2_M2_ORD_3rd };
    if (_set_config_accelerometer() != OK)
        return ERR_SPI;

    if (_set_accel_offsets(acc_offset_x, acc_offset_y, acc_offset_z) != OK)
        return ERR_SPI;

    m_gyro_cfg = { GYRO_FS_SEL_2000, GYRO_ODR_200Hz, GYRO_UI_FILT_ORD_3rd, GYRO_UI_FILT_BW_4, GYRO_DEC2_M2_ORD_3rd };
    if (_set_config_gyro() != OK)
        return ERR_SPI;

    // Configure shared filter bandwidth
    u8 filt_bw = (m_accel_cfg.accel_ui_filt_bw << 4) | m_gyro_cfg.gyro_ui_filt_bw;
    if (_write_reg(GYRO_ACCEL_CONFIG0, filt_bw) != OK)
        return ERR_SPI;

    // Precompute sensitivity scale factors
    m_accel_lsb_per_g
        = _accel_sensitivity_scale_factor(static_cast<ACCEL_FS_SEL>(m_accel_cfg.accel_fs_sel));
    m_gyro_lsb_per_degree
        = _gyro_sensitivity_scale_factor(static_cast<GYRO_FS_SEL>(m_gyro_cfg.gyro_fs_sel));

    // Power on sensors
    if (_toggle_accel_gyro_temp(DEVICE_STATUS::ON) != OK)
        return ERR_SPI;

    m_initialized = true;
    return OK;
}

int ICM40609D::_set_config_accelerometer()
{
    // ACCEL_CONFIG0
    u8 tx0[2] = { (ACCEL_CONFIG0 & 0x7F), 0 };
    tx0[1] |= m_accel_cfg.accel_fs_sel << 5;
    tx0[1] |= m_accel_cfg.accel_odr;
    if (_spi_transmit(tx0, 2) != OK)
        return ERR_SPI;
    HAL_Delay(1);

    // ACCEL_CONFIG1
    u8 tx1[2] = { (ACCEL_CONFIG1 & 0x7F), 0 };
    tx1[1] |= m_accel_cfg.accel_ui_filt_ord << 3;
    tx1[1] |= m_accel_cfg.accel_dec2_m2_ord << 1;
    if (_spi_transmit(tx1, 2) != OK)
        return ERR_SPI;
    HAL_Delay(1);

    return OK;
}

int ICM40609D::_set_config_gyro()
{
    u8 tx0[2] = { (GYRO_CONFIG0 & 0x7F), 0 };
    tx0[1] |= m_gyro_cfg.gyro_fs_sel << 5;
    tx0[1] |= m_gyro_cfg.gyro_odr;
    if (_spi_transmit(tx0, 2) != OK)
        return ERR_SPI;
    HAL_Delay(1);

    u8 tx1[2] = { (GYRO_CONFIG1 & 0x7F), 0 };
    tx1[1] |= m_gyro_cfg.gyro_ui_filt_ord << 2;
    tx1[1] |= m_gyro_cfg.gyro_dec2_m2_ord;
    if (_spi_transmit(tx1, 2) != OK)
        return ERR_SPI;
    HAL_Delay(1);

    return OK;
}

TemperatureData ICM40609D::read_temperature()
{
    if (_ensure_initialized() != OK)
        return TemperatureData { 0.f };

    u8 rx[2] = {};
    if (_spi_receive(TEMP_DATA1_UI, rx, 2) != OK) {
        return TemperatureData { 0.f };
    }

    float raw_assembled = icm_be16_to_float(rx[0], rx[1]);
    return TemperatureData { (raw_assembled / 132.48f) + 25.f };
}

AccelerationData ICM40609D::read_acceleration()
{
    if (_ensure_initialized() != OK)
        return AccelerationData { 0.f, 0.f, 0.f };

    u8 rx[6] = { 0 };
    if (_spi_receive(ACCEL_DATA_X1_UI, rx, 6) != OK)
        return AccelerationData { 0.f, 0.f, 0.f };

    float raw_assembled[3] { icm_be16_to_float(rx[0], rx[1]), icm_be16_to_float(rx[2], rx[3]), icm_be16_to_float(rx[4], rx[5]) };

    return AccelerationData { (raw_assembled[0] / m_accel_lsb_per_g) * static_cast<float>(G), (raw_assembled[1] / m_accel_lsb_per_g) * static_cast<float>(G),
        (raw_assembled[2] / m_accel_lsb_per_g) * static_cast<float>(G) };
}

GyroData ICM40609D::read_gyroscope()
{
    if (_ensure_initialized() != OK)
        return GyroData { 0.f, 0.f, 0.f };

    u8 rx[6] = {};
    if (_spi_receive(GYRO_DATA_X1_UI, rx, 6) != OK)
        return GyroData { 0.f, 0.f, 0.f };

    float raw_assembled[3] = { icm_be16_to_float(rx[0], rx[1]), icm_be16_to_float(rx[2], rx[3]), icm_be16_to_float(rx[4], rx[5]) };

    return GyroData { raw_assembled[0] / m_gyro_lsb_per_degree, raw_assembled[1] / m_gyro_lsb_per_degree,
        raw_assembled[2] / m_gyro_lsb_per_degree };
}

GyroData ICM40609D::get_last_gyroscope()
{
    return m_sensor_data.gyro;
}

AccelerationData ICM40609D::get_last_acceleration()
{
    return m_sensor_data.acc;
}

TemperatureData ICM40609D::get_last_temperature()
{
    return m_sensor_data.temperature;
}

int ICM40609D::read(ImuData& data)
{
    int status = _spi_receive_data_from_register();
    data       = m_sensor_data;
    return status;
}

int ICM40609D::sample_data()
{
    int status = _spi_receive_data_from_register();
    return status;
}

int ICM40609D::_get_device_id(u8& id)
{
    u8 rx      = 0;
    int status = _spi_receive(WHO_AM_I, &rx, 1);
    id         = rx;
    return status;
}

int ICM40609D::_ensure_initialized()
{
    if (!m_initialized)
        return init(m_accel_offset_g.acc_offset_x, m_accel_offset_g.acc_offset_y, m_accel_offset_g.acc_offset_z);

    if (_is_device_present())
        return OK;

    m_initialized = false;
    m_sensor_data.valid = false;
    return ERR_INVALID_DEVICE;
}

bool ICM40609D::_is_device_present()
{
    u8 id {};
    return _get_device_id(id) == OK && id == WHO_AM_I_RESPONSE_ID;
}

int ICM40609D::_spi_receive_data_from_register()
{
    int ret = _ensure_initialized();
    if (ret != OK) {
        m_sensor_data.valid = false;
        return ret;
    }

    u8 rx[14] = { 0 };
    if (_spi_receive(TEMP_DATA1_UI, rx, 14) != OK) {
        m_sensor_data.valid = false;
        return ERR_SPI;
    }

    // Registers are contiguous: TEMP[1:0], ACCEL[X1:X0,Y1:Y0,Z1:Z0], GYRO[X1:X0,Y1:Y0,Z1:Z0]
    float raw_assembled[7] = { icm_be16_to_float(rx[0], rx[1]), icm_be16_to_float(rx[2], rx[3]), icm_be16_to_float(rx[4], rx[5]),
        icm_be16_to_float(rx[6], rx[7]), icm_be16_to_float(rx[8], rx[9]), icm_be16_to_float(rx[10], rx[11]),
        icm_be16_to_float(rx[12], rx[13]) };

    TemperatureData temperature_in_celsius { (raw_assembled[0] / 132.48f) + 25.f };

    AccelerationData acceleration_data_m_per_ss { (raw_assembled[1] / m_accel_lsb_per_g) * static_cast<float>(G),
        (raw_assembled[2] / m_accel_lsb_per_g) * static_cast<float>(G), (raw_assembled[3] / m_accel_lsb_per_g) * static_cast<float>(G) };

    GyroData gyro_data { raw_assembled[4] / m_gyro_lsb_per_degree, raw_assembled[5] / m_gyro_lsb_per_degree,
        raw_assembled[6] / m_gyro_lsb_per_degree };

    m_sensor_data = { true, temperature_in_celsius, gyro_data, acceleration_data_m_per_ss, tx_time_get() };

    return OK;
}

int ICM40609D::_bank_select(BANK_SEL bank)
{
    u8 tx[2] = { (REG_BANK_SEL & 0x7F), static_cast<u8>(bank) };
    return _spi_transmit(tx, 2);
}

u8 ICM40609D::_get_register_value(const u8 reg)
{
    u8 rx = 0;
    _spi_receive(reg, &rx, 1);
    return rx;
}
