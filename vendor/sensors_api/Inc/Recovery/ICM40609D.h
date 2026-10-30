/**
 * @file ICM40609D.h
 *
 * @author Nikolai G. Borbe (nikolai.borbe@propulsentnu.com)
 * @brief The header file for the ICM-40609-D (IMU) driver.
 * @version 1.0
 *
 * @date 2026-03-26
 */

#ifndef ICM40609D_H
#define ICM40609D_H

#include "IMU.h"
#include "main.h"
#include "tx_api.h"
#include "borbes_shorthands.h"

#ifdef __cplusplus
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#endif


struct IcmSpiConfig {
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;
    SPI_HandleTypeDef* hspi;
};

struct IcmAccelConfig {
    u8 accel_fs_sel;
    u8 accel_odr;
    u8 accel_ui_filt_bw;
    u8 accel_ui_filt_ord;
    u8 accel_dec2_m2_ord;
};

struct IcmGyroConfig {
    u8 gyro_fs_sel;
    u8 gyro_odr;
    u8 gyro_ui_filt_ord;
    u8 gyro_ui_filt_bw;
    u8 gyro_dec2_m2_ord;
};

#ifdef __cplusplus
class ICM40609D : public IMU<ImuData> {
public:
    enum Error {
        OK = 0,
        ERR_SPI = -1,
        ERR_INVALID_DEVICE = -2,
    };

    ICM40609D(const IcmSpiConfig& spi)
        : IMU<ImuData>(), m_spi(spi) {};

private:
    ImuData m_sensor_data {};

private:
    /**
     * @brief Relevant data registers.
     */
    static constexpr u8 TEMP_DATA1_UI      = 0x1D;
    static constexpr u8 ACCEL_DATA_X1_UI   = 0x1F;
    static constexpr u8 GYRO_DATA_X1_UI    = 0x25;
    static constexpr u8 ACCEL_CONFIG0      = 0x50;
    static constexpr u8 GYRO_ACCEL_CONFIG0 = 0x52;
    static constexpr u8 ACCEL_CONFIG1      = 0x53;
    static constexpr u8 GYRO_CONFIG0       = 0x4F;
    static constexpr u8 GYRO_CONFIG1       = 0x51;
    static constexpr u8 PWR_MGMT0          = 0x4E;
    static constexpr u8 WHO_AM_I           = 0x75;
    static constexpr u8 DEVICE_CONFIG      = 0x11;
    static constexpr u8 REG_BANK_SEL       = 0x76;

    /**
     * @brief Accelerometer full-scale range options.
     */
    enum ACCEL_FS_SEL {
        ACCEL_FS_SEL_32g = 0x00,
        ACCEL_FS_SEL_16g = 0x01,
        ACCEL_FS_SEL_8g  = 0x02,
        ACCEL_FS_SEL_4g  = 0x03
    };

    enum ACCEL_UI_FILT_ORD { ACCEL_UI_FILT_ORD_1st = 0x00, ACCEL_UI_FILT_ORD_2nd = 0x01, ACCEL_UI_FILT_ORD_3rd = 0x02 };

    enum ACCEL_ODR {
        ACCEL_ODR_4kHz  = 0x04,
        ACCEL_ODR_2kHz  = 0x05,
        ACCEL_ODR_1kHz  = 0x06,
        ACCEL_ODR_200Hz = 0x07,
        ACCEL_ODR_100Hz = 0x08
    };

    enum GYRO_FS_SEL {
        GYRO_FS_SEL_2000 = 0x00,
        GYRO_FS_SEL_1000 = 0x01,
        GYRO_FS_SEL_500  = 0x02,
        GYRO_FS_SEL_250  = 0x03,
        GYRO_FS_SEL_125  = 0x04,
        GYRO_FS_SEL_63   = 0x05,
        GYRO_FS_SEL_31   = 0x06,
        GYRO_FS_SEL_16   = 0x07,
    };

    enum GYRO_ODR {
        GYRO_ODR_32kHz = 0x01,
        GYRO_ODR_16kHz = 0x02,
        GYRO_ODR_8kHz  = 0x03,
        GYRO_ODR_4kHz  = 0x04,
        GYRO_ODR_2kHz  = 0x05,
        GYRO_ODR_1kHz  = 0x06,
        GYRO_ODR_500Hz = 0x0F,
        GYRO_ODR_200Hz = 0x07,
        GYRO_ODR_100Hz = 0x08,
        GYRO_ODR_50Hz  = 0x09,
        GYRO_ODR_25Hz  = 0x0A,
        GYRO_ODR_12Hz  = 0x0B
    };

    enum GYRO_UI_FILT_ORD {
        GYRO_UI_FILT_ORD_1st = 0x00,
        GYRO_UI_FILT_ORD_2nd = 0x01,
        GYRO_UI_FILT_ORD_3rd = 0x02,
    };

    enum BANK_SEL {
        BANK_SEL_0 = 0x00,
        BANK_SEL_1 = 0x01,
        BANK_SEL_2 = 0x02,
        BANK_SEL_4 = 0x04,
    };

    enum OFFSET_USER {
        OFFSET_USER0 = 0x77,
        OFFSET_USER1 = 0x78,
        OFFSET_USER2 = 0x79,
        OFFSET_USER3 = 0x7A,
        OFFSET_USER4 = 0x7B,
        OFFSET_USER5 = 0x7C,
        OFFSET_USER6 = 0x7D,
        OFFSET_USER7 = 0x7E,
        OFFSET_USER8 = 0x7F,
    };

    // Others
    static constexpr u8 ACCEL_UI_FILT_BW_4    = 0x01;
    static constexpr u8 ACCEL_DEC2_M2_ORD_3rd = 0x02;
    static constexpr u8 GYRO_UI_FILT_BW_4     = 0x01;
    static constexpr u8 GYRO_DEC2_M2_ORD_3rd  = 0x02;
    static constexpr u8 WHO_AM_I_RESPONSE_ID  = 0x3B;

public:
    int init(float acc_offset_x = 0.f, float acc_offset_y = 0.f, float acc_offset_z = 0.f);
    int read(ImuData& data);
    AccelerationOffset read_accel_offset_registers();
    GyroData read_gyroscope();
    AccelerationData read_acceleration();
    TemperatureData read_temperature();
    int sample_data();
    GyroData get_last_gyroscope();
    AccelerationData get_last_acceleration();
    TemperatureData get_last_temperature();

private:
    int _set_config_accelerometer();
    int _set_config_gyro();
    float _accel_sensitivity_scale_factor(ACCEL_FS_SEL full_scale_select);
    float _gyro_sensitivity_scale_factor(GYRO_FS_SEL full_scale_select);
    int _spi_receive_data_from_register();
    int _get_device_id(u8& id);
    int _ensure_initialized();
    bool _is_device_present();
    int _toggle_accel_gyro_temp(DEVICE_STATUS st);
    int _soft_reset();
    int _set_offset_acc_x(float offset_g);
    int _set_offset_acc_y(float offset_g);
    int _set_offset_acc_z(float offset_g);
    int _set_accel_offsets(float offset_x_g, float offset_y_g, float offset_z_g);
    void _cs_low();
    void _cs_high();
    int _spi_transmit(u8* data, u16 len);
    int _spi_receive(u8 reg, u8* data, u16 len);
    int _write_reg(u8 reg, u8 value);
    u8 _get_register_value(const u8 reg);
    int _bank_select(BANK_SEL bank);

private:
    IcmSpiConfig m_spi;
    float m_accel_lsb_per_g;
    float m_gyro_lsb_per_degree;
    IcmAccelConfig m_accel_cfg;
    IcmGyroConfig m_gyro_cfg;
    AccelerationOffset m_accel_offset_g {};
    bool m_initialized {};
};

#endif

#endif
