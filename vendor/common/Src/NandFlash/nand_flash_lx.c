/**
 * @file lx_nand_flash.c
 * @author Paulius Juzenas (paulius.juzenas@propulse.no)
 * @brief I have no idea what I am doing.
 * @version 0.1
 * @date 2026-06-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "nand_flash_lx.h"

static OSPI_HandleTypeDef *hospi = NULL;

static UINT pages_read(ULONG block, ULONG page, UCHAR* main_buffer, UCHAR* spare_buffer, ULONG pages) {
    return (nand_flash_pages_read(hospi, (uint32_t)block, (uint32_t)page, main_buffer, spare_buffer, (uint32_t)pages) == 0) ? LX_SUCCESS : LX_ERROR;
}

static UINT pages_write(ULONG block, ULONG page, UCHAR* main_buffer, UCHAR* spare_buffer, ULONG pages) {
    return (nand_flash_pages_write(hospi, (uint32_t)block, (uint32_t)page, main_buffer, spare_buffer, (uint32_t)pages) == 0) ? LX_SUCCESS : LX_ERROR;
}

static UINT pages_copy(ULONG source_block, ULONG source_page, ULONG destination_block, ULONG destination_page, ULONG pages, UCHAR *data_buffer) {
    return (nand_flash_pages_copy(hospi,
                                 (uint32_t)source_block,
                                 (uint32_t)source_page,
                                 (uint32_t)destination_block,
                                 (uint32_t)destination_page,
                                 (uint32_t)pages,
                                 data_buffer) == 0) ? LX_SUCCESS : LX_ERROR;
}

static UINT block_erase(ULONG block, ULONG erase_count) {
    return (nand_flash_block_erase(hospi, (uint32_t)block, (uint32_t)erase_count) == 0) ? LX_SUCCESS : LX_ERROR;
}

static UINT page_erased_verify(ULONG block, ULONG page) {
    return (nand_flash_page_erased_verify(hospi, (uint32_t)block, (uint32_t)page) == 0) ? LX_SUCCESS : LX_ERROR;
}

static UINT block_erased_verify(ULONG block) {
    return (nand_flash_block_erased_verify(hospi, (uint32_t)block) == 0) ? LX_SUCCESS : LX_ERROR;
}

static UINT block_status_get(ULONG block, UCHAR *bad_block_byte) {
    return (nand_flash_block_status_get(hospi, (uint32_t)block, bad_block_byte) == 0) ? LX_SUCCESS : LX_ERROR;
}

static UINT block_status_set(ULONG block, UCHAR bad_block_byte) {
    return (nand_flash_block_status_set(hospi, (uint32_t)block, bad_block_byte) == 0) ? LX_SUCCESS : LX_ERROR;
}

static UINT system_error(UINT error_code, ULONG block, ULONG page) {
    // Fuck :(
    return LX_ERROR;
}

UINT nand_flash_lx_initialize(LX_NAND_FLASH *nand_flash) {
    if (nand_flash_command_reset(hospi) != 0)
        return LX_ERROR;

    if (nand_flash_wait_until_ready(hospi) != 0)
        return LX_ERROR;

    if (nand_flash_command_set_feature(hospi, FLASH_FEATURE_ADDRESS_BLOCK_LOCK, 0x00) != 0)
        return LX_ERROR;

    nand_flash -> lx_nand_flash_total_blocks    = NAND_FLASH_TOTAL_BLOCKS;
    nand_flash -> lx_nand_flash_pages_per_block = NAND_FLASH_PAGES_PER_BLOCK;
    nand_flash -> lx_nand_flash_bytes_per_page  = NAND_FLASH_BYTES_PER_PHYSICAL_PAGE;

    // The functions marked "NOPE" are not mentioned in the docs
    // and looking at the code they seem unused. Therefore, I
    // assume that they are not needed.

    // nand_flash -> lx_nand_flash_driver_read             = NOPE
    // nand_flash -> lx_nand_flash_driver_write            = NOPE
    nand_flash -> lx_nand_flash_driver_block_erase         = block_erase;
    nand_flash -> lx_nand_flash_driver_block_erased_verify = block_erased_verify;
    nand_flash -> lx_nand_flash_driver_page_erased_verify  = page_erased_verify;
    nand_flash -> lx_nand_flash_driver_block_status_get    = block_status_get;
    nand_flash -> lx_nand_flash_driver_block_status_set    = block_status_set;
    // nand_flash -> lx_nand_flash_driver_extra_bytes_get  = NOPE
    // nand_flash -> lx_nand_flash_driver_extra_bytes_set  = NOPE
    nand_flash -> lx_nand_flash_driver_system_error        = system_error;

    nand_flash -> lx_nand_flash_driver_pages_read  = pages_read;
    nand_flash -> lx_nand_flash_driver_pages_write = pages_write;
    nand_flash -> lx_nand_flash_driver_pages_copy  = pages_copy;

    nand_flash -> lx_nand_flash_spare_data1_offset = NAND_FLASH_SPARE_DATA1_OFFSET;
    nand_flash -> lx_nand_flash_spare_data1_length = NAND_FLASH_SPARE_DATA1_LENGTH;

    nand_flash -> lx_nand_flash_spare_data2_offset = NAND_FLASH_SPARE_DATA2_OFFSET;
    nand_flash -> lx_nand_flash_spare_data2_length = NAND_FLASH_SPARE_DATA2_LENGTH;

    nand_flash -> lx_nand_flash_spare_total_length = NAND_FLASH_SPARE_BYTES_PER_PAGE;

    return LX_SUCCESS;
}
