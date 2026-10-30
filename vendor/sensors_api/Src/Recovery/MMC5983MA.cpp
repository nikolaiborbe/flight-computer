/**
 * @file MMC5983MA.cpp
 *
 * @brief SPI driver for the MEMSIC MMC5983MA 3-axis magnetometer.
 */

#include "MMC5983MA.h"

void MMC5983MA::_cs_low() { HAL_GPIO_WritePin(m_spi.cs_port, m_spi.cs_pin, GPIO_PIN_RESET); }
void MMC5983MA::_cs_high() { HAL_GPIO_WritePin(m_spi.cs_port, m_spi.cs_pin, GPIO_PIN_SET); }

int MMC5983MA::_read_reg(u8 reg, u8& value)
{
    return _read_regs(reg, &value, 1);
}

int MMC5983MA::_read_regs(u8 start_reg, u8* data, u16 len)
{
    if (data == nullptr || len == 0)
        return ERR_INVALID_ARG;

    u8 cmd = static_cast<u8>(0x80u | (start_reg & 0x3Fu));

    _cs_low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(m_spi.hspi, &cmd, 1, MMC5983MA_SPI_TIMEOUT_MS);
    if (status == HAL_OK)
        status = HAL_SPI_Receive(m_spi.hspi, data, len, MMC5983MA_SPI_TIMEOUT_MS);
    _cs_high();

    return (status == HAL_OK) ? OK : ERR_SPI;
}

int MMC5983MA::_write_reg(u8 reg, u8 value)
{
    u8 tx[2] = { static_cast<u8>(reg & 0x3Fu), value };

    _cs_low();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(m_spi.hspi, tx, 2, MMC5983MA_SPI_TIMEOUT_MS);
    _cs_high();

    return (status == HAL_OK) ? OK : ERR_SPI;
}

int MMC5983MA::_write_control_0(u8 value)
{
    int ret = _write_reg(REG_INTERNAL_CONTROL_0, value);
    if (ret == OK)
        m_control_0 = value & CTRL0_AUTO_SR_EN;
    return ret;
}

int MMC5983MA::_write_control_1(u8 value)
{
    int ret = _write_reg(REG_INTERNAL_CONTROL_1, value);
    if (ret == OK) {
        m_control_1 = value & CTRL1_BW_MASK;
        m_bandwidth = static_cast<Bandwidth>(m_control_1 & CTRL1_BW_MASK);
    }
    return ret;
}

int MMC5983MA::_write_control_2(u8 value)
{
    int ret = _write_reg(REG_INTERNAL_CONTROL_2, value);
    if (ret == OK)
        m_control_2 = value;
    return ret;
}

int MMC5983MA::_wait_status(u8 mask, u32 timeout_ms)
{
    u32 start = HAL_GetTick();
    do {
        u8 status = 0;
        int ret = _read_reg(REG_STATUS, status);
        if (ret != OK)
            return ret;
        if ((status & mask) == mask)
            return OK;
        HAL_Delay(1);
    } while ((HAL_GetTick() - start) < timeout_ms);

    return ERR_TIMEOUT;
}

u32 MMC5983MA::_measurement_timeout_ms() const
{
    switch (m_bandwidth) {
    case Bandwidth::BW_800HZ: return 5;
    case Bandwidth::BW_400HZ: return 9;
    case Bandwidth::BW_200HZ: return 17;
    case Bandwidth::BW_100HZ:
    default: return 33;
    }
}

bool MMC5983MA::_continuous_mode_enabled() const
{
    return (m_control_2 & CTRL2_CMM_EN) != 0;
}

bool MMC5983MA::_is_device_present()
{
    if (!m_initialized)
        return false;

    u8 id = 0;
    return read_product_id(id) == OK && id == PRODUCT_ID;
}

Mmc5983maVector MMC5983MA::_raw_to_gauss(const Mmc5983maVectorRaw& raw) const
{
    return {
        (static_cast<float>(static_cast<s32>(raw.x) - static_cast<s32>(RAW_ZERO_FIELD)) / RAW_LSB_PER_GAUSS),
        (static_cast<float>(static_cast<s32>(raw.y) - static_cast<s32>(RAW_ZERO_FIELD)) / RAW_LSB_PER_GAUSS),
        (static_cast<float>(static_cast<s32>(raw.z) - static_cast<s32>(RAW_ZERO_FIELD)) / RAW_LSB_PER_GAUSS),
    };
}

int MMC5983MA::init()
{
    m_initialized = false;
    m_sensor_data.valid = false;

    _cs_high();
    HAL_Delay(10);

    u8 id = 0;
    int ret = read_product_id(id);
    if (ret != OK)
        return ret;
    if (id != PRODUCT_ID)
        return ERR_INVALID_DEVICE;

    ret = soft_reset();
    if (ret != OK)
        return ret;

    ret = read_product_id(id);
    if (ret != OK)
        return ret;
    if (id != PRODUCT_ID)
        return ERR_INVALID_DEVICE;

    ret = set_bandwidth(Bandwidth::BW_100HZ);
    if (ret != OK)
        return ret;

    ret = enable_automatic_set_reset(true);
    if (ret != OK)
        return ret;

    ret = perform_set_operation();
    if (ret != OK)
        return ret;

    m_initialized = true;
    return OK;
}

int MMC5983MA::read(Mmc5983maData& data)
{
    int ret = sample_data();
    data = m_sensor_data;
    return ret;
}

int MMC5983MA::sample_data()
{
    if (!_is_device_present()) {
        m_sensor_data.valid = false;
        return ERR_INVALID_DEVICE;
    }

    Mmc5983maVectorRaw raw {};
    int ret = read_magnetic_raw(raw);
    if (ret != OK) {
        m_sensor_data.valid = false;
        return ret;
    }

    float temperature = 0.0f;
    ret = read_temperature(temperature);
    if (ret != OK) {
        m_sensor_data.valid = false;
        return ret;
    }

    m_sensor_data = { true, _raw_to_gauss(raw), raw, temperature, HAL_GetTick() };
    return OK;
}

int MMC5983MA::read_magnetic(Mmc5983maVector& magnetic_gauss)
{
    if (!_is_device_present())
        return ERR_INVALID_DEVICE;

    Mmc5983maVectorRaw raw {};
    int ret = read_magnetic_raw(raw);
    if (ret != OK)
        return ret;

    magnetic_gauss = _raw_to_gauss(raw);
    return OK;
}

int MMC5983MA::read_magnetic_raw(Mmc5983maVectorRaw& raw)
{
    if (!_is_device_present())
        return ERR_INVALID_DEVICE;

    int ret = OK;

    if (!_continuous_mode_enabled()) {
        ret = _write_control_0(m_control_0 | CTRL0_TM_M);
        if (ret != OK)
            return ret;
    }

    ret = _wait_status(STATUS_MEAS_M_DONE, _measurement_timeout_ms());
    if (ret != OK)
        return ret;

    u8 rx[7] = {};
    ret = _read_regs(REG_XOUT0, rx, 7);
    if (ret != OK)
        return ret;

    raw.x = (static_cast<u32>(rx[0]) << 10) | (static_cast<u32>(rx[1]) << 2) | (static_cast<u32>(rx[6]) >> 6);
    raw.y = (static_cast<u32>(rx[2]) << 10) | (static_cast<u32>(rx[3]) << 2) | ((static_cast<u32>(rx[6]) >> 4) & 0x03u);
    raw.z = (static_cast<u32>(rx[4]) << 10) | (static_cast<u32>(rx[5]) << 2) | ((static_cast<u32>(rx[6]) >> 2) & 0x03u);

    return OK;
}

int MMC5983MA::read_temperature(float& temperature_c)
{
    if (!_is_device_present())
        return ERR_INVALID_DEVICE;

    int ret = _write_control_0(m_control_0 | CTRL0_TM_T);
    if (ret != OK)
        return ret;

    ret = _wait_status(STATUS_MEAS_T_DONE, 5);
    m_control_0 &= static_cast<u8>(~CTRL0_TM_T);
    if (ret != OK)
        return ret;

    u8 raw = 0;
    ret = _read_reg(REG_TOUT, raw);
    if (ret != OK)
        return ret;

    temperature_c = -75.0f + (static_cast<float>(raw) * 0.8f);
    return OK;
}

Mmc5983maData MMC5983MA::get_last_data() const { return m_sensor_data; }

Mmc5983maVector MMC5983MA::get_last_magnetic() const { return m_sensor_data.magnetic_gauss; }

float MMC5983MA::get_last_temperature() const { return m_sensor_data.temperature_c; }

int MMC5983MA::set_bandwidth(Bandwidth bandwidth)
{
    u8 bw = static_cast<u8>(bandwidth);
    if (bw > static_cast<u8>(Bandwidth::BW_800HZ))
        return ERR_INVALID_ARG;

    return _write_control_1(static_cast<u8>((m_control_1 & ~CTRL1_BW_MASK) | bw));
}

int MMC5983MA::enable_automatic_set_reset(bool enable)
{
    u8 value = m_control_0;
    if (enable)
        value |= CTRL0_AUTO_SR_EN;
    else
        value &= static_cast<u8>(~CTRL0_AUTO_SR_EN);

    return _write_control_0(value);
}

int MMC5983MA::perform_set_operation()
{
    int ret = _write_control_0(m_control_0 | CTRL0_SET);
    HAL_Delay(1);
    return ret;
}

int MMC5983MA::perform_reset_operation()
{
    int ret = _write_control_0(m_control_0 | CTRL0_RESET);
    HAL_Delay(1);
    return ret;
}

int MMC5983MA::configure_continuous_mode(ContinuousFrequency frequency, bool enable_periodic_set)
{
    u8 freq = static_cast<u8>(frequency);
    if (freq > static_cast<u8>(ContinuousFrequency::HZ_1000))
        return ERR_INVALID_ARG;

    u8 value = static_cast<u8>(m_control_2 & ~(CTRL2_CM_FREQ_MASK | CTRL2_CMM_EN | CTRL2_PRD_SET_MASK | CTRL2_EN_PRD_SET));
    value |= freq;

    if (freq != static_cast<u8>(ContinuousFrequency::OFF))
        value |= CTRL2_CMM_EN;

    if (enable_periodic_set && freq != static_cast<u8>(ContinuousFrequency::OFF))
        value |= CTRL2_EN_PRD_SET;

    if ((value & CTRL2_EN_PRD_SET) != 0) {
        int ret = enable_automatic_set_reset(true);
        if (ret != OK)
            return ret;
    }

    return _write_control_2(value);
}

int MMC5983MA::read_product_id(u8& id)
{
    return _read_reg(REG_PRODUCT_ID, id);
}

int MMC5983MA::soft_reset()
{
    int ret = _write_control_1(CTRL1_SW_RST);
    HAL_Delay(15);

    m_control_0 = 0;
    m_control_1 = 0;
    m_control_2 = 0;
    m_bandwidth = Bandwidth::BW_100HZ;
    m_sensor_data = {};

    return ret;
}
