/**
 * @file flash_nand.h
 * @author Paulius Juzenas (paulius.juzenas@propulse.no)
 * @brief A higher level flash driver for the AS5F34G04SNDB-08LIN flash chip.
 * @version 0.1
 * @date 2026-07-06
 * 
 * @copyright Copyright (c) 2026
 * 
 * The contents of this file was originally written as part of the LevelX glue.
 * However, the methods were rather helpful so were moved outside to be able to
 * be reused elsewhere.
 *
 */

#ifndef NAND_FLASH_H
#define NAND_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nand_flash_commands.h"

#define NAND_FLASH_TOTAL_BLOCKS              (4096U)
#define NAND_FLASH_PAGES_PER_BLOCK           (64U)
#define NAND_FLASH_BYTES_PER_PHYSICAL_PAGE   (2048U)
#define NAND_FLASH_SPARE_BYTES_PER_PAGE      (64U)

#define NAND_FLASH_BAD_BLOCK_POSITION        (0U)
#define NAND_FLASH_SPARE_DATA1_OFFSET        (1U)
#define NAND_FLASH_SPARE_DATA1_LENGTH        (4U)
#define NAND_FLASH_SPARE_DATA2_OFFSET        (5U)
#define NAND_FLASH_SPARE_DATA2_LENGTH        (2U)

#define NAND_FLASH_STATUS_POLL_LIMIT         (100000U)

int nand_flash_calculate_row_address(uint32_t block, uint32_t page, uint32_t *row_address);

int nand_flash_wait_until_ready(OSPI_HandleTypeDef *hospi);

int nand_flash_check_program_failure(OSPI_HandleTypeDef *hospi);
int nand_flash_check_erase_failure(OSPI_HandleTypeDef *hospi);
int nand_flash_check_ecc_failure(OSPI_HandleTypeDef *hospi);

int nand_flash_init(OSPI_HandleTypeDef *hospi);

int nand_flash_enable_write(OSPI_HandleTypeDef *hospi);
int nand_flash_program_page(OSPI_HandleTypeDef *hospi, uint32_t row_address, uint8_t *main_buffer, uint8_t *spare_buffer);
int nand_flash_pages_read(OSPI_HandleTypeDef *hospi, uint32_t block, uint32_t page, uint8_t *main_buffer, uint8_t *spare_buffer, uint32_t pages);
int nand_flash_pages_write(OSPI_HandleTypeDef *hospi, uint32_t block, uint32_t page, uint8_t *main_buffer, uint8_t *spare_buffer, uint32_t pages);
int nand_flash_pages_copy(OSPI_HandleTypeDef *hospi, uint32_t source_block, uint32_t source_page, uint32_t destination_block, uint32_t destination_page, uint32_t pages, uint8_t *data_buffer);

int nand_flash_block_erase(OSPI_HandleTypeDef *hospi, uint32_t block, uint32_t erase_count);
int nand_flash_page_erased_verify(OSPI_HandleTypeDef *hospi, uint32_t block, uint32_t page);
int nand_flash_block_erased_verify(OSPI_HandleTypeDef *hospi, uint32_t block);

int nand_flash_block_status_get(OSPI_HandleTypeDef *hospi, uint32_t block, uint8_t *bad_block_byte);
int nand_flash_block_status_set(OSPI_HandleTypeDef *hospi, uint32_t block, uint8_t bad_block_byte);

#ifdef __cplusplus
}
#endif

#endif /* NAND_FLASH_H */