
/**************************************************************************/
/*                                                                        */
/*       Copyright (c) Microsoft Corporation. All rights reserved.        */
/*                                                                        */
/*       This software is licensed under the Microsoft Software License   */
/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
/*       and in the root directory of this software.                      */
/*                                                                        */
/**************************************************************************/

#ifndef FX_PROPULSE_SD_DRIVER_H
#define FX_PROPULSE_SD_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "fx_api.h"
#include "sdmmc.h"
#include "stm32h7xx_hal.h"

extern TX_SEMAPHORE propulse_sd_tx_semaphore;
extern TX_SEMAPHORE propulse_sd_rx_semaphore;

/* Timeout used to wait for fx operations */
#define FX_PROPULSE_SD_TIMEOUT (10*TX_TIMER_TICKS_PER_SECOND)

/* Enable the cache mainatenance, required when using the SD DMA */
#define FX_PROPULSE_SD_CACHE_MAINTENANCE 0

/* Sector size, used by the driver */
#define FX_PROPULSE_SD_SECTOR_SIZE 512

/* Size of the bounce scratch buffer in sectors. Larger batches keep unaligned
 * transfers multi-block instead of degrading to one DMA per sector. */
#define FX_PROPULSE_SD_SCRATCH_SECTORS 32 /* 16 kB */

/* Size of the bounce scratch buffer in bytes. */
#define FX_PROPULSE_SD_SCRATCH_SIZE (FX_PROPULSE_SD_SCRATCH_SECTORS * FX_PROPULSE_SD_SECTOR_SIZE)

/* Define the CPU Cache maintenance macros */
#if (FX_PROPULSE_SD_CACHE_MAINTENANCE == 1)
#define invalidate_cache_by_addr(__ptr__, __size__)                                                \
  SCB_InvalidateDCache_by_Addr((void *)__ptr__, (int32_t)__size__)
#define clean_cache_by_addr(__ptr__, __size__)                                                     \
  SCB_CleanDCache_by_Addr((uint32_t *)__ptr__, (int32_t)__size__)
#endif

/* Define the macro to get the current time in ticks */
#define FX_PROPULSE_SD_CURRENT_TIME() tx_time_get()

/* Macro called before initializing the SD driver
 * e.g. create a semaphore used for transfer notification */
#define FX_PROPULSE_SD_PRE_INIT(_media_ptr)                                                        \
  do {                                                                                             \
    if ((tx_semaphore_create(&propulse_sd_rx_semaphore, "sd rx transfer semaphore", 0) != TX_SUCCESS) ||    \
        (tx_semaphore_create(&propulse_sd_tx_semaphore, "sd tx transfer semaphore", 0) != TX_SUCCESS)) {    \
      _media_ptr->fx_media_driver_status = FX_IO_ERROR;                                            \
    }                                                                                              \
  } while (0)

/* Macro called after the SD deinit */
#define FX_PROPULSE_SD_POST_DEINIT(_media_ptr)                                                     \
  do {                                                                                             \
    tx_semaphore_delete(&propulse_sd_rx_semaphore);                                                         \
    tx_semaphore_delete(&propulse_sd_tx_semaphore);                                                         \
  } while (0)

/* Define how to notify about Read completion operation */
#define FX_PROPULSE_SD_READ_CPLT_NOTIFY()                                                          \
  do {                                                                                             \
    if (tx_semaphore_get(&propulse_sd_rx_semaphore, FX_PROPULSE_SD_TIMEOUT) != TX_SUCCESS) {        \
      return FX_IO_ERROR;                                                                          \
    }                                                                                              \
  } while (0)

/* Define how to notify about write completion operation */
#define FX_PROPULSE_SD_WRITE_CPLT_NOTIFY()                                                         \
  do {                                                                                             \
    if (tx_semaphore_get(&propulse_sd_tx_semaphore, FX_PROPULSE_SD_TIMEOUT) != TX_SUCCESS) {        \
      return FX_IO_ERROR;                                                                          \
    }                                                                                              \
  } while (0)

INT fx_propulse_sd_get_status(void);

INT fx_propulse_sd_read_blocks(UINT *buffer, UINT start_block, UINT total_blocks);
INT fx_propulse_sd_write_blocks(UINT *buffer, UINT start_block, UINT total_blocks);

VOID fx_propulse_sd_driver(FX_MEDIA *media_ptr);

#ifdef __cplusplus
}
#endif

#endif /* FX_PROPULSE_SD_DRIVER_H */
