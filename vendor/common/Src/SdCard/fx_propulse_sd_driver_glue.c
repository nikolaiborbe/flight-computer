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

#include "fx_propulse_sd_driver.h"

TX_SEMAPHORE propulse_sd_tx_semaphore;
TX_SEMAPHORE propulse_sd_rx_semaphore;

extern SD_HandleTypeDef hsd1;

/**
 * @brief Check the SD IP status.
 * @retval 0 when ready 1 when busy
 */
INT fx_propulse_sd_get_status(void) {
  HAL_SD_CardStateTypeDef state = HAL_SD_GetCardState(&hsd1);
  return state != HAL_SD_CARD_TRANSFER;
}

/**
 * @brief Read Data from the SD device into a buffer.
 * @param UINT *buffer buffer into which the data is to be read.
 * @param UINT start_block the first block to start reading from.
 * @param UINT total_blocks total number of blocks to read.
 * @retval 0 on success error code otherwise
 */
INT fx_propulse_sd_read_blocks(UINT *buffer, UINT start_block, UINT total_blocks) {
  return HAL_SD_ReadBlocks_DMA(&hsd1, (uint8_t *)buffer, start_block, total_blocks);
}

/**
 * @brief Write data buffer into the SD device.
 * @param UINT *buffer buffer to write into the SD device.
 * @param UINT start_block the first block to start writing into.
 * @param UINT total_blocks total number of blocks to write.
 * @retval 0 on success error code otherwise
 */
INT fx_propulse_sd_write_blocks(UINT *buffer, UINT start_block, UINT total_blocks) {
  return HAL_SD_WriteBlocks_DMA(&hsd1, (uint8_t *)buffer, start_block, total_blocks);
}

/**
 * @brief SD DMA Tx Transfer completed callbacks
 * @param Instance the sd instance
 * @retval None
 */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd) { tx_semaphore_put(&propulse_sd_tx_semaphore); }

/**
 * @brief SD DMA Rx Transfer completed callbacks
 * @param Instance the sd instance
 * @retval None
 */
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd) { tx_semaphore_put(&propulse_sd_rx_semaphore); }
