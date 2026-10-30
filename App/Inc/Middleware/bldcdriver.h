/**
 * @file bldcdriver.h
 * @authors Simen Bjerkestrand (Simen.Bjerkestrand@propulsentnu.no)
 * @brief Driver for bldc control
 * @version 1.0
 * @date 2023-11-03
 *
 * @copyright Propulse NTNU (c) 2023
 *
 */

#ifndef __BLDCDRIVER_H__
#define __BLDCDRIVER_H__

/*****************************************************************************/
/** ANSI C libraries                                                        **/
/*****************************************************************************/
#include <stdint.h>

/*****************************************************************************/
/** Module includes                                                         **/
/*****************************************************************************/
#include "fdcandriver.h"
#include "main.h"

/*****************************************************************************/
/** Module defines                                                          **/
/*****************************************************************************/
#define STATUS uint32_t

#define BLDC_MOV_OPEN 100
#define BLDC_MOV_CLOSED 0
#define BLDC_MFV_OPEN 100
#define BLDC_MFV_CLOSED 0

#define BLDC_MOV_MAX 10000
#define BLDC_MOV_MIN 0
#define BLDC_MFV_MAX 10000
#define BLDC_MFV_MIN 0

#define BLDC_MAX_CURRENT 20

#define BLDC_MOV_VALVE_ADDRESS 0x11
#define BLDC_MFV_VALVE_ADDRESS 0x22

typedef struct {
    FDCAN_HandleTypeDef* hfdcan;
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t isInitialized;
    uint32_t address;
    uint32_t position;
    uint32_t current;
} BLDC_HandleTypeDef;

typedef struct CAN_Feedback { // 64Byte data
    uint32_t Status_mode;
    uint32_t Status_warning;
    uint32_t Status_faults;
    float Status_setpoint;

    float Position_Encoder1_pos;   // 0 to 360 000
    float Position_Encoder2_pos;   // 0 to 360 000
    float Position_Calculated_pos; //-2,147,483,648 to 2,147,483,647
    float Position_Velocity;       //-2,147,483,648 to 2,147,483,647

    float Current_Q;
    float Current_D;

    int16_t Temp_ENCODER1;
    int16_t Temp_ENCODER2;

    float Voltage_BAT;
    float Voltage_AUX;

    uint8_t recerve[12];
} CAN_Feedback;

typedef struct CAN_Status {
    uint32_t status;
    float setpoint;
} CAN_Status;

//--------------------LIMITS---------------------------------
typedef enum {
    INPUT_CTRL_OFF,
    INPUT_CTRL_VOLTAGE_Q_SETPOINT,
    INPUT_CTRL_VOLTAGE_D_SETPOINT,
    INPUT_CTRL_CURRENT_Q_SETPOINT,
    INPUT_CTRL_CURRENT_D_SETPOINT,
    INPUT_CTRL_VELOCITY_SETPOINT,
    INPUT_CTRL_ANGLE_SETPOINT,
    INPUT_CTRL_ZERO_GRAVITY,

    INPUT_SET_CURRENT_Q_kp,
    INPUT_SET_CURRENT_Q_ki,
    INPUT_SET_CURRENT_Q_kd,
    INPUT_SET_CURRENT_D_kp,
    INPUT_SET_CURRENT_D_ki,
    INPUT_SET_CURRENT_D_kd,
    INPUT_SET_VELOCITY_kp,
    INPUT_SET_VELOCITY_ki,
    INPUT_SET_VELOCITY_kd,
    INPUT_SET_ANGLE_kp,
    INPUT_SET_ANGLE_ki,
    INPUT_SET_ANGLE_kd,

    INPUT_GET_CURRENT_Q_kp,
    INPUT_GET_CURRENT_Q_ki,
    INPUT_GET_CURRENT_Q_kd,
    INPUT_GET_CURRENT_D_kp,
    INPUT_GET_CURRENT_D_ki,
    INPUT_GET_CURRENT_D_kd,
    INPUT_GET_VELOCITY_kp,
    INPUT_GET_VELOCITY_ki,
    INPUT_GET_VELOCITY_kd,
    INPUT_GET_ANGLE_kp,
    INPUT_GET_ANGLE_ki,
    INPUT_GET_ANGLE_kd,

    INPUT_SET_VOLTAGE_LIMIT,
    INPUT_SET_CURRENT_LIMIT,
    INPUT_SET_VELOCITY_LIMIT,
    INPUT_SET_ANGLE_MIN_LIMIT,
    INPUT_SET_ANGLE_MAX_LIMIT,

    INPUT_GET_VOLTAGE_LIMIT,
    INPUT_GET_CURRENT_LIMIT,
    INPUT_GET_VELOCITY_LIMIT,
    INPUT_GET_ANGLE_MIN_LIMIT,
    INPUT_GET_ANGLE_MAX_LIMIT,

    INPUT_SET_MOTOR_KV,
    INPUT_GET_MOTOR_KV,

    INPUT_SET_ENCODER_1_OFFSET,
    INPUT_SET_ENCODER_2_OFFSET,
    INPUT_GET_ENCODER_1_OFFSET,
    INPUT_GET_ENCODER_2_OFFSET,

    INPUT_SET_ENCODER_2_GEAR_RATIO,
    INPUT_GET_ENCODER_2_GEAR_RATIO,

    INPUT_SAVE_TO_FLASH,

    INPUT_CALIBRATE,
    INPUT_RESET_ERRORS

} BLDC_STATUS_INPUT_t;

typedef enum {
    BLDC_STOPPED_WITH_BREAK,
    BLDC_CALIBRATING_ENCODER,
    BLDC_RUNNING,
    BLDC_ERROR
} BLDC_STATUS_Feedback;

static char status_string[5][26]
    = {"  BLDC STOPPED WITH BREAK\0", "BLDC STOPPED AND SHUTDOWN\0", " BLDC CALIBRATING ENCODER\0",
       "             BLDC RUNNING\0", "               BLDC ERROR\0"};

extern CAN_Feedback* Feedback;
/*****************************************************************************/
/** Module status codes                                                     **/
/*****************************************************************************/
#define BLDCDRIVER_STATUS_OK 0x00
#define BLDCDRIVER_STATUS_ERROR 0x01
#define BLDCDRIVER_STATUS_ALREADY_INITIALIZED 0x02

/*****************************************************************************/
/** Public API                                                              **/
/*****************************************************************************/

STATUS BLDC_init(BLDC_HandleTypeDef* bldc);

STATUS BLDC_set_position(BLDC_HandleTypeDef* bldc, float pos);

STATUS BLDC_set_current_limit(BLDC_HandleTypeDef* bldc, float limit);

STATUS BLDC_off(BLDC_HandleTypeDef* bldc);

STATUS BLDC_save_to_flash(BLDC_HandleTypeDef* bldc);

void BLDC_process_data(uint8_t RxData[64]);

#endif /* __BLDCDRIVER_H__ */