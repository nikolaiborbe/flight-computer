/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_threadx.c
 * @author  MCD Application Team
 * @brief   ThreadX applicative file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2020-2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
void app_setup(void);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

// uint8_t tracex_buffer[TRACEX_BUFFER_SIZE] __attribute__ ((section (".trace")));
// the memory addresses for the tracex buffer are found in build/Debug/FC_nucleo.map, and then
// Ctrl+F ".trace" run the command under DEBUG CONSOLE: dump binary memory memory.bin 0x240400c8
// 0x240500c8
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
#define BOOTSTRAP_STACK_SIZE   2048u          /* size to what app_setup needs */
#define BOOTSTRAP_PRIORITY     1u             /* high prio: run setup before app threads */

static TX_THREAD bootstrap_thread;
static uint8_t stack[BOOTSTRAP_STACK_SIZE];

static VOID bootstrap_thread_entry(ULONG arg)
{
    (void)arg;
    app_setup();
}
/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  /* USER CODE BEGIN App_ThreadX_MEM_POOL */
  (void)memory_ptr; /* bootstrap stack is static now, pool unused */
  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */
    ret = tx_thread_create(
        &bootstrap_thread,
        "bootstrap",
        bootstrap_thread_entry,
        0u,
        stack,
        BOOTSTRAP_STACK_SIZE,
        BOOTSTRAP_PRIORITY,          /* priority */
        BOOTSTRAP_PRIORITY,          /* preempt-threshold */
        TX_NO_TIME_SLICE,
        TX_AUTO_START
    );
  /* USER CODE END App_ThreadX_Init */

  return ret;
}

  /**
  * @brief  Function that implements the kernel's initialization.
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */

  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
