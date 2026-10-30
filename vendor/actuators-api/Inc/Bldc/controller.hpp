/**
 * @file controller.hpp
 * @author Vetle R. Dahl (vetle.dahl@propulsentnu.no)
 * @brief electronic pressure regulator controller class
 * @version 1.0
 * @date 2025-12-21
 *
 * @copyright Propulse NTNU (c) 2025
 */

/*****************************************************************************/
/* Module includes                                                           */
/*****************************************************************************/

#pragma once

#include "config.h"
#include "moteus.hpp"
#include "moteus_protocol.hpp"
#include "pcan.hpp"
#include "pressure_sensor.hpp"
#include "servo.hpp"
#include "strategy.hpp"
#include "timebase.hpp"
#include "tx_api.h"
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>

/*
  TODO: Make a working calibration function/routine. (Done? -Paulius)
  TODO: Change the way the motor reacts to offset between valve and motor
 */

namespace actuators::bldc {
/**
 * @brief Class for  for managing motor control and active control loop of the electronic regulator
 * system.
 */
class BldcController {
  public:
    /**
     * @brief Constructor for the BldcController class. Initializes the controller with the given
     * configuration and sets up the CAN communication and control strategies.
     *
     * The CAN ID for the moteus controller is our ID (source) left shifted 1 byte OR-ed with the
     * moteus controller's ID (destination). E.g moteus_id = 1 and our_id = 0 gives CAN ID = (0 <<
     * 8) | (1) = 1. The moteus controller sends responses in the same format, but with the two IDs
     * swapped places. E.g. moteus_id = 1 and our_id = 0 gives CAN ID = (1 << 8) | (0) = 256.
     *
     * @param controller_config Configuration parameters for the controller
     * @param pcan Reference to the PCan object for CAN communication
     * @param our_id ID of our selves.
     * @param moteus_id ID of the moteus controller.
     */
    BldcController(ControllerConfig controller_config, PCan &pcan, uint8_t our_id,
                   uint8_t moteus_id)
        : current_strategy(std::make_unique<ManualStrategy>(controller_config)),
          controller_config(controller_config), our_id(our_id), moteus_id(moteus_id), pcan(pcan),
          moteus_instance(Moteus(pcan,
                                 [moteus_id]() {
                                     Moteus::Options options;
                                     options.id = moteus_id;
                                     options.query_format.extra[0].register_number =
                                         mm::Register::kPower;
                                     options.query_format.extra[0].resolution = mm::kFloat;
                                     options.query_format.extra[1].register_number =
                                         mm::Register::kEncoder1Position;
                                     options.query_format.extra[1].resolution = mm::kFloat;

                                     return options;
                                 }())),
          moteus_msg_max_age(controller_config.can_watchdog_timeout),
          gear_ratio(controller_config.gear_ratio), max_torque(controller_config.max_torque),
          valve_encoder_offset(controller_config.valve_encoder_offset),
          min_pos(controller_config.minimum_position), max_pos(controller_config.maximum_position) {
    }

    int init() {
        return pcan.subscribe_raw<BldcController>(moteus_id << 8 | our_id,
                                                  PCAN_EXTENDED_CAN_ID_MASK, this,
                                                  &BldcController::can_callback);
    }

#pragma pack(push, 1)
    /**
     * @brief Struct containing telemetry data from the BLDC controller.
     */
    struct BldcTelemetry {
        float valve_position;
        float motor_position;
        float current_draw;
        float u_p;
        float u_i;
        float u_d;
        float u_total;
        float u_raw;
        uint8_t status_flags; // Bit 0: Calibrated, Bit 1: Has timed out
    } telemetry_payload;
#pragma pack(pop)

    /**
     * @brief Get the telemetry data from the controller, including sensor measurements and control
     * outputs.
     *
     * @return BldcTelemetry struct containing the latest telemetry data.
     */
    BldcTelemetry get_telemetry() {
        const LockGuard g(bldc_controller_lock, Mutex::WaitForever);
        telemetry_payload.valve_position      = valve_position;
        telemetry_payload.motor_position      = motor_position;
        telemetry_payload.current_draw        = current_draw;
        Strategy::CtrlOutput strategy_output  = current_strategy->get_output();
        telemetry_payload.u_p                 = strategy_output.u_p;
        telemetry_payload.u_i                 = strategy_output.u_i;
        telemetry_payload.u_d                 = strategy_output.u_d;
        telemetry_payload.u_total             = strategy_output.u_pid + strategy_output.u_ff;
        telemetry_payload.u_raw               = u_set;
        telemetry_payload.status_flags        = (calibrated * 1)  | ((has_timed_out * 1) << 1) | ((static_cast<uint8_t>(ctrl_result.mode) & 0xF) << 4);
        return telemetry_payload;
    }

    /**
     * @brief Binds the given servo and pressure sensor to the controller, allowing it to read
     * pressure values and servo encoder values for control calculations.
     *
     * @param servo_downstream
     * @param pressure_upstream
     * @param pressure_downstream
     */
    void bind(actuators::servo::Servo &servo_downstream,
              sensors::pressure::PressureSensor &pressure_upstream,
              sensors::pressure::PressureSensor &pressure_downstream) noexcept {
        downstream_servo           = &servo_downstream;
        upstream_pressure_sensor   = &pressure_upstream;
        downstream_pressure_sensor = &pressure_downstream;
    }

    /**
     * @brief Main control loop tick function. Increments the control loop one time step, updating
     * measurements and calculating new position command.
     */
    int tick() {
        const LockGuard g(bldc_controller_lock, Mutex::WaitForever);

        previous_timestamp = timestamp;
        timestamp          = timebase::micros();

        if (update_measurements() < 0) {
            return -1;
        }

        bool should_query_data = (timestamp - last_query_time_us) >= query_interval_us;
        if (should_query_data) {
            last_query_time_us = timestamp;

            if ((timestamp - moteus_msg_timestamp) >= moteus_msg_max_age) {
                has_timed_out = true;
                // Invalidate motor setup
                calibrated = false;
                // Recover the moteus controller.
                moteus_instance.BeginStop();
                return 0;
            }

            moteus_instance.BeginQuery();
        }

        // !!!!!!!!!!!!!!!!!!!!!
        // !! SAFETY CRITICAL !!
        // !!!!!!!!!!!!!!!!!!!!!
        //
        // It is unsafe to send any commands to the moteus controller if we don't know where it is.
        if (has_timed_out)
            return 0;

        bool should_send_position_command =
            (timestamp - last_position_command_time_us) >= position_command_interval_us;
        if (should_send_position_command) {
            last_position_command_time_us = timestamp;

            u_set = current_strategy->calculate_position(ctrl_input);

            std::optional<mm::PositionMode::Command> command = set_position(u_set);

            if (command.has_value()) {
                if (abs(valve_position - command->position) > 0.001f) {
                    moteus_instance.BeginPosition(*command);
                }
            } else {
                moteus_instance.BeginStop();
            }
        }

        return 0;
    };

    /**
     * @brief Updates the ctrl_input struct for use in control strategies.
     */
    int update_measurements() {
        const LockGuard g(bldc_controller_lock, Mutex::WaitForever);

        if (upstream_pressure_sensor == nullptr || downstream_pressure_sensor == nullptr ||
            downstream_servo == nullptr) {
            // This **should** never happen.
            return -1;
        }

        ctrl_input.upstream_pressure   = upstream_pressure_sensor->getPressure();
        ctrl_input.downstream_pressure = downstream_pressure_sensor->getPressure();
        ctrl_input.dt =
            std::clamp((timestamp - previous_timestamp) / 1000000.0, 0.005,
                       0.015); // Placeholder for timestep, clamped to avoid unsafe values
        ctrl_input.servo_encoder_value =
            (std::isnan(downstream_servo->getStatus().measured_angle_deg)
                 ? 0
                 : downstream_servo->getStatus().measured_angle_deg) /
            downstream_servo->getModel()
                .angle_limits.max_deg; // Placeholder for valve position reading
        ctrl_input.valve_position = valve_position;
        ctrl_input.motor_position = motor_position;

        return 0;
    }

    /**
     * @brief Set the BLDC motor position.
     */
    std::optional<mm::PositionMode::Command> set_position(float desired_valve_position) {
        const LockGuard g(bldc_controller_lock, Mutex::WaitForever);
        if (ctrl_result.mode != mm::Mode::kPosition && ctrl_result.mode != mm::Mode::kStopped && ctrl_result.mode != mm::Mode::kCalibrating && ctrl_result.mode != mm::Mode::kEnabling) {
            return std::nullopt;
        }

        mm::PositionMode::Command controller_position_command = {
            .position = 0.0, .velocity = 0.0, .maximum_torque = max_torque, .watchdog_timeout = -1};

        controller_position_command.position = valve_calibration_correction;
        if (!calibrated) {
            controller_position_command.position +=
                desired_valve_position; // map 0-1 to 0-25 motor position + offset to calibrate valve position +
                          // extra to account for the difference between motor and valve position
        } else {
            controller_position_command.position += std::clamp(
                desired_valve_position, min_pos,
                max_pos); // map 0-1 to 0-25 motor position + offset to calibrate valve position
        }

        controller_position_command.position *= gear_ratio;

        if (controller_position_command.position - previous_cmd_position - 0.005 > 0) {
            direction = 1;
        } else if (controller_position_command.position - previous_cmd_position + 0.005 < 0) {
            direction = -1;
        }

        if (abs(valve_position - desired_valve_position) > 0.005f) {
            controller_position_command.maximum_torque = max_torque;
            controller_position_command.accel_limit    = 2000;
            controller_position_command.velocity_limit = 230;

        } else {
            controller_position_command.maximum_torque = max_torque / 2.0f;
            controller_position_command.accel_limit    = 50;
            controller_position_command.velocity_limit = 20;
        }
       
        previous_cmd_position = controller_position_command.position;
       
        if (calibrated) {
            if (direction == -1) { // hysteresis compensation
                controller_position_command.position -= hysteresis_offset;
            }
        }

        return controller_position_command;
    }

    /**
     * @brief Set the controller strategy to manual mode with a specified position.
     */
    void set_strategy_manual(float position = 0.0f) {
        const LockGuard g(bldc_controller_lock, Mutex::WaitForever);
        current_strategy = std::make_unique<ManualStrategy>(controller_config);
        current_strategy->set_parameters(std::clamp(position, min_pos, max_pos));
    }

    /**
     * @brief Set the controller strategy to calibration mode.

    * @param position
     */
    void set_strategy_calibrate(float position = 0.0f) {
        const LockGuard g(bldc_controller_lock, Mutex::WaitForever);

        calibrated        = false;
        hysteresis_offset = 0;

        std::unique_ptr<CalibrationStrategy<BldcController>> calibrate_strategy =
            std::make_unique<CalibrationStrategy<BldcController>>(
                controller_config, this, &BldcController::calibration_done_callback);

        calibrate_strategy->set_parameters(std::clamp(position, min_pos, max_pos));
        current_strategy = std::move(calibrate_strategy);
    }
    /**
     * @brief Set the controller strategy to pressurize mode with specified parameters.
     */
    void const set_strategy_pressurize() {
        const LockGuard g(bldc_controller_lock, Mutex::WaitForever);
        std::unique_ptr<PressurizeStrategy> pressurize_strategy =
            std::make_unique<PressurizeStrategy>(controller_config);
        pressurize_strategy->set_parameters(ctrl_input);
        current_strategy = std::move(pressurize_strategy);
    }

    /**
     * @brief Set the controller strategy to run mode with specified parameters.
     */
    void const set_strategy_run() {
        const LockGuard g(bldc_controller_lock, Mutex::WaitForever);
        std::unique_ptr<RunStrategy> run_strategy =
            std::make_unique<RunStrategy>(controller_config);
        run_strategy->set_parameters(u_set);
        current_strategy = std::move(run_strategy);
    }

    /**
     * @brief does the same as set_strategy_manual, sets bldc controller to given position
     *
     * @param position
     */
    void begin_position(float position) { set_strategy_manual(position); }

    /**
     * @brief callback function to update the controller class with message information. Called by
     * PCAN when a new message is received.
     *
     * @param message pointer to the received CAN message, containing the latest measurements and
     * status from the motor controller.
     */
    void can_callback(pcan_message_t *message) {
        const LockGuard g(bldc_controller_lock, Mutex::WaitForever);
        moteus_msg_timestamp = timebase::micros();

        // Using moteus callback parsing
        moteus_instance.rx_callback(message);
        ctrl_result = moteus_instance.last_result().values;

        valve_position = std::remainder(ctrl_result.extra[1].value * 4 - valve_encoder_offset, 4.0);
        motor_position = ctrl_result.position / gear_ratio;

        power_draw   = ctrl_result.extra[0].value;
        current_draw = power_draw / ctrl_result.voltage;

        if (has_timed_out) {
            // !!!!!!!!!!!!!!!!!!!!!
            // !! SAFETY CRITICAL !!
            // !!!!!!!!!!!!!!!!!!!!!
            //
            // If this is the first time we hear from the motor since we timed out,
            // we have noe guarantee for where it is. The safest thing to do then
            // is just to let the motor be where it is.
            //
            // Currently, this is done a bit
            // of a janky way by setting the valve calibration correction to the motor position.
            valve_calibration_correction = motor_position;

            // Reset timeout
            has_timed_out = false;
        }
    };

    void calibration_done_callback(double new_valve_calibration_correction,
                                   double new_hysteresis_offset) {
        const LockGuard g(bldc_controller_lock, Mutex::WaitForever);
        valve_calibration_correction += new_valve_calibration_correction;
        hysteresis_offset = 0; //new_hysteresis_offset * gear_ratio;
        current_strategy  = std::make_unique<ManualStrategy>(controller_config);
        current_strategy->set_parameters(0);
        calibrated = true;
    }

    /**
     * @brief Get the current valve position.
     *
     * @return double Current valve position.
     */
    double get_valve_position() { return valve_position; };

    /**
     * @brief Get the calibration status.
     *
     * @return bool Calibration status.
     */
    bool get_calibrated() { return calibrated; };

  private:
    // lock / mutexes
    Mutex bldc_controller_lock{};

    // Control and config
    Strategy::CtrlInput ctrl_input;
    // ManualStrategy manual_strategy;
    // PressurizeStrategy pressurize_strategy;
    // RunStrategy run_strategy;
    // CalibrationStrategy calibrate_strategy;
    std::unique_ptr<Strategy> current_strategy;

    ControllerConfig controller_config;

    const uint8_t our_id;
    const uint8_t moteus_id;
    PCan &pcan;

    // moteus spesific structs
    mm::Query::Result ctrl_result = {};
    Moteus moteus_instance;

    // attributes
    float u_set     = 0;
    bool calibrated = false;

    uint64_t previous_timestamp       = 0;
    uint64_t timestamp                = 0;
    uint64_t moteus_msg_timestamp     = 0;
    const uint64_t moteus_msg_max_age = 0; // microseconds

    uint64_t last_query_time_us                 = 0;
    static constexpr uint64_t query_interval_us = 10000; // 100 Hz

    uint64_t last_position_command_time_us                 = 0;
    static constexpr uint64_t position_command_interval_us = 10000; // 100 Hz

    double motor_position = 0;
    double valve_position = 0;

    const float gear_ratio;
    const double max_torque;

    double valve_calibration_correction = 0.0;
    double hysteresis_offset =
        1.45; // compensation for hysteresis in the valve, added to the motor position to get a more
              // accurate estimate of the valve position

    const double valve_encoder_offset; // constant offset from 0 on controller

    double power_draw   = 0.0;
    double current_draw = 0.0;

    const float min_pos;
    const float max_pos;

    int8_t direction = 1;

    double previous_cmd_position = 0.0;
    bool has_timed_out           = true;

    // bound servo and pressure sensor references
    actuators::servo::Servo *downstream_servo = nullptr;

    sensors::pressure::PressureSensor *upstream_pressure_sensor   = nullptr;
    sensors::pressure::PressureSensor *downstream_pressure_sensor = nullptr;
};
} // namespace actuators::bldc
