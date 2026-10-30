/**
 * @file flash_lx.h
 * @author Paulius Juzenas (paulius.juzenas@propulse.no)
 * @brief LevelX glue code for our flash driver.
 * @version 0.1
 * @date 2026-06-11
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef FLASH_LX_H
#define FLASH_LX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nand_flash.h"
#include "lx_api.h"

/**
 * @brief Initalizes a LevelX NAND flash struct with our flash driver.
 * 
 * @param nand_flash The LevelX NAND flash struct to initalize.
 * @return UINT LevelX error code.
 */
UINT nand_flash_lx_initialize(LX_NAND_FLASH *nand_flash);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_LX_H */
