/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "borbes_shorthands.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI2_CS_ICM_Pin GPIO_PIN_2
#define SPI2_CS_ICM_GPIO_Port GPIOE
#define SPI2_CS_MMC_Pin GPIO_PIN_4
#define SPI2_CS_MMC_GPIO_Port GPIOE
#define BUZZER_Pin GPIO_PIN_5
#define BUZZER_GPIO_Port GPIOE
#define ETH_nRST_Pin GPIO_PIN_0
#define ETH_nRST_GPIO_Port GPIOA
#define DUMMY_ADC_PIN_Pin GPIO_PIN_4
#define DUMMY_ADC_PIN_GPIO_Port GPIOA
#define BURNWIRE_Pin GPIO_PIN_1
#define BURNWIRE_GPIO_Port GPIOB
#define STORAGE_CD_Pin GPIO_PIN_12
#define STORAGE_CD_GPIO_Port GPIOE
#define HEADER_PD11_Pin GPIO_PIN_11
#define HEADER_PD11_GPIO_Port GPIOD
#define HEADER_PD13_Pin GPIO_PIN_13
#define HEADER_PD13_GPIO_Port GPIOD
#define USER_BUTTON_Pin GPIO_PIN_15
#define USER_BUTTON_GPIO_Port GPIOD
#define LED_USER2_Pin GPIO_PIN_4
#define LED_USER2_GPIO_Port GPIOD
#define LED_USER1_Pin GPIO_PIN_5
#define LED_USER1_GPIO_Port GPIOD
#define LED_STOR_Pin GPIO_PIN_6
#define LED_STOR_GPIO_Port GPIOD
#define LED_ETH_Pin GPIO_PIN_8
#define LED_ETH_GPIO_Port GPIOB
#define LED_FDCAN2_Pin GPIO_PIN_9
#define LED_FDCAN2_GPIO_Port GPIOB
#define LED_FDCAN1_Pin GPIO_PIN_0
#define LED_FDCAN1_GPIO_Port GPIOE
#define PYRO_EN_Pin GPIO_PIN_1
#define PYRO_EN_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
