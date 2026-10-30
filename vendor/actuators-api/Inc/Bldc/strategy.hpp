#pragma once

#include "tx_api.h"
#include "mappings.hpp"
#include "mutex.hpp"

#include <cmath>
#include <algorithm>

namespace actuators::bldc {

struct PidGains {
  double p = 1.0;
  double i = 0;
  double d = 0;
  double normalization_factor = 1.0;
  bool integral_clamp = false;
  double integral_clamp_max = 0.01;
  double integral_clamp_min = -0.01;

};
  
/**
 * @brief Configuration struct for the Controller class.
 */
struct ControllerConfig {
  PidGains pressurize_pid_gains;
  PidGains run_pid_gains;
  double minimum_position = 0;
  double maximum_position = 1;
  double valve_encoder_offset;
  uint64_t can_watchdog_timeout = 1000000;
  double end_pressure_ref = 30.0;
  double pressurizing_time = 15.0;
  float gear_ratio = 25.0;
  double max_torque = 0.16;
  double dvdt_nominal = 1.0;
  double cv_max = 1.6;
  SigmoidParameters control_valve_curve_pressurization;
  SigmoidParameters control_valve_curve_run;
  SigmoidParameters main_valve_curve;
};



enum class CALIBRATION_PHASE {
    CALIBRATION_PHASE_1,
    CALIBRATION_PHASE_2,
    CALIBRATION_PHASE_3,
    CALIBRATION_DONE
};

/**
 * @brief  The strategy class is a base class for different control strategies for how the valve position is to be moved.
 *
 * @param config controller configuration struct
 */
class Strategy {
  public:
    Strategy(ControllerConfig& config) :
    min_position{config.minimum_position},
    max_position{config.maximum_position},
    end_pressure_ref{config.end_pressure_ref},
    control_valve_curve_pressurization(config.control_valve_curve_pressurization),
    control_valve_curve_run(config.control_valve_curve_run),
    main_valve_curve(config.main_valve_curve)
    {
      current_pressure_ref = start_pressure_ref<end_pressure_ref?start_pressure_ref:end_pressure_ref;
    };
  #pragma pack(push, 1)
  /**
  * @brief struct for new relevant information into the strategies
  *
  */
  struct CtrlInput {
    double upstream_pressure;
    double downstream_pressure;
    double dt;
    double valve_position;
    double motor_position;
    double servo_encoder_value;
  };

  struct CtrlOutput{
    double u_p = 0;
    double u_i = 0;
    double u_d = 0;
    double u_pid = 0;
    double u_ff = 0;
  } ctrl_output;
  #pragma pack(pop)

    /**
    * @brief destructor
    */
    virtual ~Strategy() {};

    /**
     * @brief Calculates the desired valve position based on the current measurements and the specific control strategy implemented in the derived class.
     *
     * @param ctrl_input updated struct from the controller class containing the latest measurements and relevant information for control.
     * @return double wanted valve position
     */
    virtual double calculate_position(CtrlInput& ctrl_input) = 0;


    /**
     * @brief Set the object parameters
     *
     * @param position
     */
    void set_parameters(float position){
      const LockGuard g(bldc_strategy_lock, Mutex::WaitForever);
      valve_set_position = position;
    }

    /**
     * @brief get spesific data from the strategy for telemetry or other uses
     */
    CtrlOutput get_output(){
      const LockGuard g(bldc_strategy_lock, Mutex::WaitForever);
      return ctrl_output;
    };

  protected:
    /**
      * @brief Update the strategy attributes from the controller input
      *
      * @param ctrl_input
      */
    void update_measurements(CtrlInput &ctrl_input){
      const LockGuard g(bldc_strategy_lock, Mutex::WaitForever);
      upstream_pressure = ctrl_input.upstream_pressure;
      downstream_pressure = ctrl_input.downstream_pressure;
      dt = ctrl_input.dt;
      servo_encoder_value = ctrl_input.servo_encoder_value;
      valve_set_position = ctrl_input.valve_position;
      motor_position = ctrl_input.motor_position;

    }

    /**
     * @brief Classic pid loop, using internal attributes
     *
     * @return double
     */
    double calculate_pid() {
      const LockGuard g(bldc_strategy_lock, Mutex::WaitForever);
      error = current_pressure_ref-downstream_pressure;
      // PID calculations
      ctrl_output.u_p = pid_gains.p * error / pid_gains.normalization_factor;
      ctrl_output.u_i += pid_gains.i * (error * dt) / pid_gains.normalization_factor;
      if (pid_gains.integral_clamp){
      ctrl_output.u_i = std::clamp(ctrl_output.u_i,pid_gains.integral_clamp_min,pid_gains.integral_clamp_max);
      }
      ctrl_output.u_d = pid_gains.d * ((error - previous_error) / dt) / pid_gains.normalization_factor;
      ctrl_output.u_pid = ctrl_output.u_p + ctrl_output.u_i + ctrl_output.u_d;

      previous_error = error;
      return ctrl_output.u_pid;
    };

    double dt = 0.01f;
    double valve_set_position = 0.0;
    double motor_position = 0.0;
    double min_position = 0.0;
    double max_position = 1.0;
    double servo_encoder_value = 0.0;
    double upstream_pressure = 0.0;
    double downstream_pressure = 0.0;
    double start_pressure_ref = 0.0;
    double current_pressure_ref = 0.0;
    double end_pressure_ref = 0.0;
    double error = 0.0;
    double previous_error = 0.0;
    double pressurizing_time = 1.0;
    SigmoidParameters control_valve_curve_pressurization;
    SigmoidParameters control_valve_curve_run;
    SigmoidParameters main_valve_curve;
    PidGains pid_gains;

    Mutex bldc_strategy_lock{};
};

class PressurizeStrategy : public Strategy {
  public:
    explicit PressurizeStrategy(ControllerConfig& config) :
    Strategy(config),
    pressurizing_time{config.pressurizing_time}
    {
      pid_gains = config.pressurize_pid_gains;
    };

    ~PressurizeStrategy() override {};

    /**
     * @brief Set the correct start pressure reference for pressurizing objective
     *
     * @param ctrl_input
     */
    void set_parameters(CtrlInput &ctrl_input){
      update_measurements(ctrl_input);
      const LockGuard g(bldc_strategy_lock, Mutex::WaitForever);
      start_pressure_ref = fmax(downstream_pressure, 0);
      current_pressure_ref = start_pressure_ref<end_pressure_ref?start_pressure_ref:end_pressure_ref;
    };

    /**
     * @brief calculates a desired valve position and sends it to the controller
     *
     * @param ctrl_input
     * @return double
     */
    double calculate_position(CtrlInput &ctrl_input) override {
      update_measurements(ctrl_input);

      //linearly increase pressure reference
      if (current_pressure_ref <end_pressure_ref){
        current_pressure_ref += (dt/pressurizing_time) * (end_pressure_ref - start_pressure_ref);
      }
      else {
          current_pressure_ref = end_pressure_ref;
      }

      ctrl_output.u_pid = calculate_pid();
      
      // If shit broke comment out this part...
      double valve_pos = logit_curve(std::clamp(ctrl_output.u_pid, 0.0, 1.0), 0.0, 1.0, min_position, max_position, control_valve_curve_pressurization);
      return std::clamp(valve_pos, min_position, max_position);

      // ...and uncomment this part.
      // return std::clamp(ctrl_output.u_pid,min_position,max_position);
    };
  private:
  double pressurizing_time;
};

class RunStrategy : public Strategy {
  public:
    explicit RunStrategy(ControllerConfig& config) :
    Strategy(config),
    dvdt_nominal{config.dvdt_nominal},
    cv_max{config.cv_max}
    {
      pid_gains = config.run_pid_gains;
    };

    ~RunStrategy() override {};

    /**
     * @brief Set the values back to default for the run strategy, and set the position from the pressurize strategy to ensure a smooth transition between the two strategies.
     *
     * @param ctrl_input
     */
    void set_parameters(float u_set){
      const LockGuard g(bldc_strategy_lock, Mutex::WaitForever);
      position_from_pressurize = u_set;
      current_pressure_ref = end_pressure_ref;
      previous_ff = 0.0f;
    };

    /**
     * @brief calculates a desired valve position and sends it to the controller
     *
     * @param ctrl_input
     * @return double
     */
    double calculate_position(CtrlInput &ctrl_input) override {
      //update
      update_measurements(ctrl_input);
      //position calculation
      ctrl_output.u_pid = calculate_pid();
      ctrl_output.u_ff = additive_feed_forward();
      if (ctrl_output.u_ff < previous_ff) {
        ctrl_output.u_ff = previous_ff;
      }
      previous_ff = ctrl_output.u_ff;
      return std::clamp(ctrl_output.u_pid+ctrl_output.u_ff,min_position,max_position);
    };

  protected:
    /**
     * @brief function using the system state and a physics model to predict the required valve position to achieve the desired mass flow, used as an additive feedforward term to the PID output.
     *
     * @return const float
     */
    const float additive_feed_forward()
    {
      // Assuming all temperatures are 293 K.
      
      // No valve opening if servo encoder is not open.
      if (abs(servo_encoder_value) < 0.01f) {
        return position_from_pressurize;
      } 
      
      // Fully open valve when DP is zero or end_pressure_ref > upstream_pressure, within some margin.
      if (upstream_pressure - end_pressure_ref < 0.1f) {
        return 1.0f;
      }
      
      // Flow rate calculation using Emerson nomenclature and equation
      float x = (upstream_pressure - end_pressure_ref) / upstream_pressure;  // Pressure drop ratio

      float y = fmax(2.0f / 3.0f, 1.0f - 2.0f / 3.0f * x);  // Expansion factor

      float cv_required = (3600.0f * 0.33678f * end_pressure_ref * dvdt_nominal / 293.0f)  // mdot in kg/h
        / (94.8f * upstream_pressure * y * sqrt(x * 28.014f / 293.0f));  // final bracket gives gpm

      // No need to check cv_required < 0, as all terms are positive
      if (cv_required >= cv_max) {
        // Required Cv exceeds maximum Cv, fully open valve
        return 1.0f;

      }
      // Convert required Cv to valve opening [0..1]
      float z = cv_required / cv_max;
      
      // Assumed opening characteristic: 1/(1+exp(-25*(z-0.6)))
      double steady_state_valve_opening = logit_curve(z, 0, 1, 0, 1, control_valve_curve_run);

      // Clamp value in case it is out of bounds.
      double clamped_valve_opening = std::clamp(steady_state_valve_opening, min_position, max_position);

      // Since the model assumes steady state it doesn't take into account the output valve opening.
      // We jinx this by simply interpolating to the predicted steady state valve opening from the pressurized steady state valvew opening.
      double interpolated_valve_opening = sigmoid_curve(servo_encoder_value, 0.0f, 1.0f, position_from_pressurize, clamped_valve_opening, main_valve_curve);

      return std::clamp(interpolated_valve_opening, position_from_pressurize, max_position);  // Limit between 0 and 1
    }

    double position_from_pressurize = 0.0;
    double previous_ff = 0.0;
    double dvdt_nominal;
    double cv_max;
};

class ManualStrategy : public Strategy {
  public:
    explicit ManualStrategy(ControllerConfig& config) :
    Strategy(config) {};
    ~ManualStrategy() override {};

    double calculate_position(CtrlInput &) override{return valve_set_position;};
};

template<typename T>
class CalibrationStrategy : public Strategy {
  public:
    explicit CalibrationStrategy(ControllerConfig& config, T *object, void (T::*calibration_done_callback)(double new_valve_calibration_correction, double new_hysteresis_offset)) : 
    Strategy(config),
    object(object),
    calibration_done_callback(calibration_done_callback)
    {};
    ~CalibrationStrategy() override {};
    double calculate_position(CtrlInput &ctrl_input) override{
      update_measurements(ctrl_input);

      if(valve_set_position>-0.04 && current_calibration_phase == CALIBRATION_PHASE::CALIBRATION_PHASE_1) {
          valve_calibration_correction -=0.001;
      }
      else if(valve_set_position<-0.08 && current_calibration_phase == CALIBRATION_PHASE::CALIBRATION_PHASE_1) {// Tunes the valve position estimate to be zero when fully closed
          valve_calibration_correction +=0.001;
      }
      else if(current_calibration_phase == CALIBRATION_PHASE::CALIBRATION_PHASE_1) {
          current_calibration_phase = CALIBRATION_PHASE::CALIBRATION_PHASE_2;
          hysteresis_offset = motor_position;
          temp_valve_position = valve_set_position;
      }

      if (valve_set_position< temp_valve_position+0.01 && current_calibration_phase == CALIBRATION_PHASE::CALIBRATION_PHASE_2) {
          valve_calibration_correction +=0.001;
      }
      else if (current_calibration_phase == CALIBRATION_PHASE::CALIBRATION_PHASE_2) {
          current_calibration_phase = CALIBRATION_PHASE::CALIBRATION_PHASE_3;
          hysteresis_offset = motor_position - hysteresis_offset-0.01; // compensation for hysteresis in the valve, added to the motor position to get a more accurate estimate of the valve position
      }


      else if(valve_set_position<-0.001 && current_calibration_phase == CALIBRATION_PHASE::CALIBRATION_PHASE_3) {// Tunes the valve position estimate to be zero when fully closed
          valve_calibration_correction +=0.001;
      }
      else if (current_calibration_phase == CALIBRATION_PHASE::CALIBRATION_PHASE_3) {
          current_calibration_phase = CALIBRATION_PHASE::CALIBRATION_DONE;
          (object->*calibration_done_callback)(valve_calibration_correction, hysteresis_offset);
         // valve_calibration_correction -= 0.001;
          return 0.0;
      }
      return valve_calibration_correction;
    };

  private:
    CALIBRATION_PHASE current_calibration_phase = CALIBRATION_PHASE::CALIBRATION_PHASE_1;
    double valve_calibration_correction = 0.0;
    double hysteresis_offset = 0.0;
    double temp_valve_position = 0.0;
    T *object = nullptr;
    void (T::*calibration_done_callback)(double new_valve_calibration_correction, double new_hysteresis_offset);

};
}
