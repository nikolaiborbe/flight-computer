#ifndef TELEMETRY_GROUNDSTATION_DATA_HPP
#define TELEMETRY_GROUNDSTATION_DATA_HPP

#include "ICM40609D.h"
#include "MMC5983MA.h"
#include "Shared.hpp"
#include "stdint.h"

#pragma pack(push, 1)
enum ChargingStatus : uint8_t {
    NotCharging       = 0x00,
    TrickleCharge     = 0x01,
    PreCharge         = 0x02,
    FastCharge        = 0x03,
    TaperCharge       = 0x04,
    Reserved          = 0x05,
    TopOffTimerCharge = 0x06,
    ChargeTermination = 0x07,
};

struct RailMeasurement {
    float current;       /**< A, signed (sign follows IN+ -> IN-)   */
    float volts;         /**< V, bus voltage at the IN- node (rail) */
    float shunt_voltage; /**< V, drop across the shunt resistor     */
    float power;         /**< W                                      */
};

struct RailData {
    // INA236 rail monitors
    RailMeasurement vbat;
    RailMeasurement v_12;
    RailMeasurement v_24;
};

struct BatteryPackData {
    float cell[6];          /**< per-cell voltage, V (NaN if not present) */
    float voltage;          /**< sum of valid cells, V                    */
    float current;          /**< A, discharge negative                    */
    float die_temperature;  /**< BQ76925 internal temperature, degC       */
    uint8_t balancing_mask; /**< BAL_1..BAL_6 currently driven            */
    bool alert;             /**< over-current comparator / ALERT pin      */
    bool comms_ok;          /**< last AFE transaction succeeded           */
};

struct ChargerData {
    double vac;         /**< adapter voltage, V        */
    double vbat;        /**< battery voltage, V        */
    double vsys;        /**< system voltage, V         */
    double iac;         /**< adapter current, A        */
    double ibat;        /**< battery current, A        */
    double temperature; /**< battery NTC temp, degC    */
    bool charging;
    bool power_good;
    uint8_t faults; /**< raw FAULT_STATUS (REG0x24) */
    ChargingStatus state;
};

enum class STATE_NAME {
    NONE,
    /* On ground */
    IDLE,
    READY,
    FILLING,
    SEPARATION_FILLING,
    FLOTATION_FILLING,

    /* EMERGENCY */
    OPEN_SEPARATION_SERVO,

    /* Flight */
    THRUSTING,
    COASTING,
    SEPARATION,
    FLOTATION,
    SHUTDOWN,
};

namespace Telemetry::GroundStation {
// sensors
typedef struct {
    uint8_t data_valid;
    uint8_t numSV;
    double lat;
    double lon;
    double hMSL;
    double gSpeed;
    double headMot;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
} GNSS_DATA;

typedef struct {
    uint8_t valid;
    float temperature; // Celsius
    float pressure;    // hPa
    float humidity;    // %RH
    uint32_t timestamp;
} BME280_DATA;

typedef struct {
    uint8_t valid;
    float temperature; // Celsius
    float pressure;    // mbar
    uint32_t timestamp;
} MS5611_DATA;

typedef struct {
    float x;
    float y;
    float z;
} MMC5983MA_DATA;

typedef struct {
    Telemetry::Shared::AccelerationData acc;
    Telemetry::Shared::GyroData gyro;
} STIM377H_DATA;

typedef struct {
    Telemetry::Shared::AccelerationData acc_m_per_ss;
    Telemetry::Shared::GyroData gyro;
    float temperature_C;
} ICM40609D_DATA;

typedef struct {
    float flotation_pressure;
    float separation_pressure;
} PressureData;

typedef struct {
    float temp1;
} ThermocoupleData;

// status
typedef struct {
    bool bme_online;
    bool ms_online;
    bool gnss_online;
    bool icm_online;
    bool mmc_online;
    bool stim_online;
    bool pressure_online;
} SensorStatus;

typedef struct {
    bool separation_filling_online;
    bool flotation_filling_online;
    bool flotation_online;
    bool separation_servo_online;
} ActuatorStatus;

typedef struct {
    bool pcan1_online;
    bool pcan2_online;
    bool networking_thread_online;
    bool kalman_thread_online;
    bool sensor_thread_online;
    bool fsm_thread_online;
} HardwareStatus;

/**
 * @telemetry
 * #port=3001
 */
typedef struct {
    GNSS_DATA gnss_data;
    BME280_DATA bme_data;
    MS5611_DATA ms_data;
    Telemetry::Shared::ImuData icm_data;
    Telemetry::Shared::MagnetometerData mmc_data;
    STIM377H_DATA stim_data;
    PressureData pressure_data;
} SensorData;

/**
 * @telemetry
 * #port=3002
 */
typedef struct {
    double altitude;
    double velocity;
} KalmanData;

/**
 * @telemetry
 * #port=3003
 */
typedef struct {
    ActuatorStatus actuators;
    SensorStatus sensors;
    HardwareStatus hardware;
    STATE_NAME current_state;
} SystemStatus;

/**
 * @telemetry
 * #port=6967
 */
typedef struct {
    ChargerData charger;
    BatteryPackData battery;
    RailData rail;
} BMS_Data;
} // namespace Telemetry::GroundStation

#pragma pack(pop)
#endif
