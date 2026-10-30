/**
 * @file leveldriver.h
 * @authors Simen Bjerkestrand (Simen.Bjerkestrand@propulsentnu.no)
 * @brief Driver for level control
 * @version 1.0
 * @date 2023-11-03
 *
 * @copyright Propulse NTNU (c) 2023
 *
 */

#ifndef __LEVELDRIVER_H__
#define __LEVELDRIVER_H__

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

typedef struct {
    FDCAN_HandleTypeDef* hfdcan;
    FDCAN_TxHeaderTypeDef TxHeader;
    uint8_t isInitialized;
    uint32_t address;
} LEVEL_HandleTypeDef;

typedef struct LEVEL_Status {
    float Channel1;
    float Channel2;
} LEVEL_Status;

extern LEVEL_Status level_status;

/*****************************************************************************/
/** Module status codes                                                     **/
/*****************************************************************************/
#define LEVELDRIVER_STATUS_OK 0x00
#define LEVELDRIVER_STATUS_ERROR 0x01
#define LEVELDRIVER_STATUS_ALREADY_INITIALIZED 0x02

/*****************************************************************************/
/** Public API                                                              **/
/*****************************************************************************/

STATUS LEVEL_init(LEVEL_HandleTypeDef* level);

void LEVEL_process_data(uint8_t RxData[64]);

#endif /* __LEVELDRIVER_H__ */