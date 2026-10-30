/**
 * @file MMC5983MA.h
 *
 * @brief SPI driver for the MEMSIC MMC5983MA 3-axis magnetometer.
 *
 * SPI notes: the device supports up to 10 MHz. The datasheet timing describes
 * CPOL=1/CPHA=1, but common breakout-library validation uses normal 4-wire,
 * MSB-first transfers with read commands formed as 0x80 | register.
 */

#ifndef MMC5983MA_H
#define MMC5983MA_H

#include "borbes_shorthands.h"
#include "main.h"

#ifdef __cplusplus

constexpr u32 MMC5983MA_SPI_TIMEOUT_MS = 100;

struct Mmc5983maSpiConfig {
    GPIO_TypeDef* cs_port;
    u16 cs_pin;
    SPI_HandleTypeDef* hspi;
};

struct Mmc5983maVectorRaw {
    u32 x;
    u32 y;
    u32 z;
};

struct Mmc5983maVector {
    float x;
    float y;
    float z;
};

struct Mmc5983maData {
    bool valid;
    Mmc5983maVector magnetic_gauss;
    Mmc5983maVectorRaw raw;
    float temperature_c;
    u32 timestamp_ms;
};

class MMC5983MA {
public:
    enum Error {
        OK = 0,
        ERR_SPI = -1,
        ERR_INVALID_DEVICE = -2,
        ERR_TIMEOUT = -3,
        ERR_INVALID_ARG = -4,
    };

    enum class Bandwidth : u8 {
        BW_100HZ = 0x00,
        BW_200HZ = 0x01,
        BW_400HZ = 0x02,
        BW_800HZ = 0x03,
    };

    enum class ContinuousFrequency : u8 {
        OFF = 0x00,
        HZ_1 = 0x01,
        HZ_10 = 0x02,
        HZ_20 = 0x03,
        HZ_50 = 0x04,
        HZ_100 = 0x05,
        HZ_200 = 0x06,
        HZ_1000 = 0x07,
    };

    explicit MMC5983MA(const Mmc5983maSpiConfig& spi)
        : m_spi(spi) {};

    int init();
    int read(Mmc5983maData& data);
    int sample_data();

    int read_magnetic(Mmc5983maVector& magnetic_gauss);
    int read_magnetic_raw(Mmc5983maVectorRaw& raw);
    int read_temperature(float& temperature_c);

    Mmc5983maData get_last_data() const;
    Mmc5983maVector get_last_magnetic() const;
    float get_last_temperature() const;

    int set_bandwidth(Bandwidth bandwidth);
    int enable_automatic_set_reset(bool enable);
    int perform_set_operation();
    int perform_reset_operation();
    int configure_continuous_mode(ContinuousFrequency frequency, bool enable_periodic_set = false);
    int read_product_id(u8& id);
    int soft_reset();

private:
    static constexpr u8 REG_XOUT0 = 0x00;
    static constexpr u8 REG_TOUT = 0x07;
    static constexpr u8 REG_STATUS = 0x08;
    static constexpr u8 REG_INTERNAL_CONTROL_0 = 0x09;
    static constexpr u8 REG_INTERNAL_CONTROL_1 = 0x0A;
    static constexpr u8 REG_INTERNAL_CONTROL_2 = 0x0B;
    static constexpr u8 REG_PRODUCT_ID = 0x2F;

    static constexpr u8 PRODUCT_ID = 0x30;

    static constexpr u8 STATUS_MEAS_M_DONE = 1u << 0;
    static constexpr u8 STATUS_MEAS_T_DONE = 1u << 1;

    static constexpr u8 CTRL0_TM_M = 1u << 0;
    static constexpr u8 CTRL0_TM_T = 1u << 1;
    static constexpr u8 CTRL0_SET = 1u << 3;
    static constexpr u8 CTRL0_RESET = 1u << 4;
    static constexpr u8 CTRL0_AUTO_SR_EN = 1u << 5;

    static constexpr u8 CTRL1_BW_MASK = 0x03;
    static constexpr u8 CTRL1_SW_RST = 1u << 7;

    static constexpr u8 CTRL2_CM_FREQ_MASK = 0x07;
    static constexpr u8 CTRL2_CMM_EN = 1u << 3;
    static constexpr u8 CTRL2_PRD_SET_MASK = 0x70;
    static constexpr u8 CTRL2_EN_PRD_SET = 1u << 7;

    static constexpr u32 RAW_ZERO_FIELD = 131072u;
    static constexpr float RAW_LSB_PER_GAUSS = 16384.0f;

    void _cs_low();
    void _cs_high();
    int _read_reg(u8 reg, u8& value);
    int _read_regs(u8 start_reg, u8* data, u16 len);
    int _write_reg(u8 reg, u8 value);
    int _write_control_0(u8 value);
    int _write_control_1(u8 value);
    int _write_control_2(u8 value);
    int _wait_status(u8 mask, u32 timeout_ms);
    u32 _measurement_timeout_ms() const;
    bool _continuous_mode_enabled() const;
    bool _is_device_present();
    Mmc5983maVector _raw_to_gauss(const Mmc5983maVectorRaw& raw) const;

private:
    Mmc5983maSpiConfig m_spi;
    Mmc5983maData m_sensor_data {};
    u8 m_control_0 {};
    u8 m_control_1 {};
    u8 m_control_2 {};
    Bandwidth m_bandwidth { Bandwidth::BW_100HZ };
    bool m_initialized {};
};

#endif

#endif
