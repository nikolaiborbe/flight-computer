/**
 * @file flash.h    
 * @author Paulius Juzenas (paulius.juzenas@propulse.no)
 * @brief Simple driver for the AS5F34G04SNDB-08LIN flash chip.
 * @version 0.1
 * @date 2026-06-08
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef FLASH_COMMANDS_H
#define FLASH_COMMANDS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#define FLASH_MANUFACTURER_ID (0x52)
#define FLASH_DEVICE_ID_32 (0x41)
#define FLASH_DEVICE_ID_34 (0x42) // This is the one we currently have.

#define FLASH_FEATURE_ADDRESS_BLOCK_LOCK (0xA0)
#define FLASH_FEATURE_ADDRESS_OTP        (0xB0)
#define FLASH_FEATURE_ADDRESS_STATUS     (0xC0)

#define FLASH_FEATURE_BLOCK_LOCK_BRWD (1 << 7)
#define FLASH_FEATURE_BLOCK_LOCK_BP2  (1 << 5)
#define FLASH_FEATURE_BLOCK_LOCK_BP1  (1 << 4)
#define FLASH_FEATURE_BLOCK_LOCK_BP0  (1 << 3)
#define FLASH_FEATURE_BLOCK_LOCK_INV  (1 << 2)
#define FLASH_FEATURE_BLOCK_LOCK_CMP  (1 << 1)

#define FLASH_FEATURE_OTP_OTP_PRT (1 << 7)
#define FLASH_FEATURE_OTP_OTP_EN  (1 << 6)
#define FLASH_FEATURE_OTP_ECC_EN  (1 << 4)
#define FLASH_FEATURE_OTP_QE      (1 << 0)

#define FLASH_FEATURE_STATUS_ECCS1  (1 << 5)
#define FLASH_FEATURE_STATUS_ECCS0  (1 << 4)
#define FLASH_FEATURE_STATUS_P_FAIL (1 << 3)
#define FLASH_FEATURE_STATUS_E_FAIL (1 << 2)
#define FLASH_FEATURE_STATUS_WEL    (1 << 1)
#define FLASH_FEATURE_STATUS_OIP    (1 << 0)

// Column Address: The 12-bit address is capable of addressing from 0 to 4095 bytes; however, only bytes 0 through 2111 are valid. 
// Bytes 2112 through 4095 of each page are “out of bounds” do not exist in the device, and cannot be addressed.
// Row Address (RA): RA<5:0> selects a page inside a block, and RA<17:6> selects a block.

int nand_flash_command_write_enable(OSPI_HandleTypeDef *hospi);
int nand_flash_command_write_disable(OSPI_HandleTypeDef *hospi);

int nand_flash_command_get_feature(OSPI_HandleTypeDef *hospi, uint8_t feature_address, uint8_t *out_feature);
int nand_flash_command_set_feature(OSPI_HandleTypeDef *hospi, uint8_t feature_address, uint8_t feature);

int nand_flash_command_read_id(OSPI_HandleTypeDef *hospi, uint8_t *out_manufacturer_id, uint8_t *out_device_id);

int nand_flash_command_page_read_to_cache(OSPI_HandleTypeDef *hospi, uint32_t row_address);
int nand_flash_command_read_from_cache_x1(OSPI_HandleTypeDef *hospi, uint16_t column_address, uint8_t *out_data, uint16_t out_data_length);

int nand_flash_command_program_load(OSPI_HandleTypeDef *hospi, uint16_t column_address, uint8_t *data, uint16_t data_length);
int nand_flash_command_program_execute(OSPI_HandleTypeDef *hospi, uint32_t row_address);

int nand_flash_command_block_erase(OSPI_HandleTypeDef *hospi, uint32_t row_address);

int nand_flash_command_reset(OSPI_HandleTypeDef *hospi);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_COMMANDS_H */
