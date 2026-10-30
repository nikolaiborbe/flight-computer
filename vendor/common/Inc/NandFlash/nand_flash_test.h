/**
 * @file nand_flash_test.h
 * @author Paulius Juzenas (paulius.juzenas@propulse.no)
 * @brief Simple test function to verify NAND flash drivers functionality.
 * @version 0.1
 * @date 2026-07-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef FLASH_TEST_H
#define FLASH_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nand_flash.h"

/**
 * @brief Simple function which test all the functions of the flash library in sequence. Aborts at the first failure. Error code is how far it got before it had to abort (negative).
 * 
 * @param hospi 
 * @return int 
 */
int nand_flash_test(OSPI_HandleTypeDef *hospi);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_TEST_H */
