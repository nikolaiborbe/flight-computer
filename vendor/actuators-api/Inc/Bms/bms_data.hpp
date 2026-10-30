#pragma once

#include <stdint.h>

// TODO: BMS is not really a sensor is it?
namespace sensors::bms {

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
#pragma pack(pop)

} // namespace Bms
