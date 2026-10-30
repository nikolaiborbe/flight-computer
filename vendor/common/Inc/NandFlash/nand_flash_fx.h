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

#ifndef NAND_FLASH_FX_H
#define NAND_FLASH_FX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "fx_api.h"
#include "lx_api.h"
#include "stm32h7xx_hal.h"
#include "nand_flash_lx.h"

/**
 * @brief Glue code between our LevelX flash driver and FileX.
 *
 * @warning This function should not be passed directly to `fx_media_open` or `fx_media_format`!
 * 
 * To use this function, make a wrapper with the call signature `void (*)(FX_MEDIA *media_ptr)`
 * where you call this function in its body with appropriate arguments. Example:
 *
 * @code{.c}
 * #include "lx_api.h"
 * #include "nand_flash_fx.h"
 * #include "octospi.h" 
 *
 * LX_NAND_FLASH my_nand_flash;
 * 
 * VOID my_fx_nand_flash_driver(FX_MEDIA *media_ptr) {
 *   return nand_flash_fx_driver(media_ptr, &my_nand_flash, &hospi2, "My Supercalifragilisticexpialidocious Flash Driver")
 * }
 * @endcode
 *
 * @param media_ptr 
 * @param lx_nand_flash 
 * @param hospi 
 * @param name 
 * @return VOID 
 */
VOID  nand_flash_fx_driver(FX_MEDIA *media_ptr, LX_NAND_FLASH *lx_nand_flash, OSPI_HandleTypeDef *hospi, char *name);

#ifdef __cplusplus
}
#endif

#endif /*NAND_FLASH_FX_H*/
