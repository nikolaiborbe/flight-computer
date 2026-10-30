#include "nand_flash.h"
#include <string.h>

int nand_flash_calculate_row_address(uint32_t block, uint32_t page, uint32_t *row_address) {
    if (!row_address || block >= NAND_FLASH_TOTAL_BLOCKS || page >= NAND_FLASH_PAGES_PER_BLOCK)
        return -1;

    *row_address = (block * NAND_FLASH_PAGES_PER_BLOCK) + page;
    return 0;
}

int nand_flash_wait_until_ready(OSPI_HandleTypeDef *hospi) {
    for (uint32_t attempts = 0; attempts < NAND_FLASH_STATUS_POLL_LIMIT; attempts++) {
        uint8_t status = 0;

        if (nand_flash_command_get_feature(hospi, FLASH_FEATURE_ADDRESS_STATUS, &status) != 0)
            return -1;

        if ((status & FLASH_FEATURE_STATUS_OIP) == 0)
            return 0;
    }

    return -1;
}

int nand_flash_check_program_failure(OSPI_HandleTypeDef *hospi) {
    uint8_t status = 0;

    if (nand_flash_command_get_feature(hospi, FLASH_FEATURE_ADDRESS_STATUS, &status) != 0)
        return -1;

    if (status & FLASH_FEATURE_STATUS_P_FAIL)
        return -1;

    return 0;
}

int nand_flash_check_erase_failure(OSPI_HandleTypeDef *hospi) {
    uint8_t status = 0;

    if (nand_flash_command_get_feature(hospi, FLASH_FEATURE_ADDRESS_STATUS, &status) != 0)
        return -1;

    if (status & FLASH_FEATURE_STATUS_E_FAIL)
        return -1;

    return 0;
}

int nand_flash_check_ecc_failure(OSPI_HandleTypeDef *hospi) {
    uint8_t status = 0;

    if (nand_flash_command_get_feature(hospi, FLASH_FEATURE_ADDRESS_STATUS, &status) != 0)
        return -1;

    /* ECC status bits: ECCS1 (bit5), ECCS0 (bit4)
       00b = no bit errors
       01b = bit error detected and corrected
       10b = bit error detected and NOT corrected
       11b = bit error detected and corrected (error bit number = ECC max)
    */
    uint8_t ecc = (status >> 4) & 0b11;

    switch (ecc) {
        case 0b00: return  0;
        case 0b01: return  0;
        case 0b10: return -1;
        case 0b11: return  0;
        default:   return -1;
    }
}

int nand_flash_init(OSPI_HandleTypeDef *hospi) {
    if (nand_flash_command_reset(hospi) != 0)
        return -1;

    if (nand_flash_wait_until_ready(hospi) != 0)
        return -1;

    if (nand_flash_command_set_feature(hospi, FLASH_FEATURE_ADDRESS_BLOCK_LOCK, 0x00) != 0)
        return -1;

    return 0;
}

int nand_flash_enable_write(OSPI_HandleTypeDef *hospi) {
    uint8_t status = 0;

    if (nand_flash_command_write_enable(hospi) != 0)
        return -1;

    if (nand_flash_command_get_feature(hospi, FLASH_FEATURE_ADDRESS_STATUS, &status) != 0)
        return -1;

    if ((status & FLASH_FEATURE_STATUS_WEL) == 0)
        return -1;

    return 0;
}

int nand_flash_program_page(OSPI_HandleTypeDef *hospi, uint32_t row_address, uint8_t *main_buffer, uint8_t *spare_buffer) {
    uint8_t buffer[NAND_FLASH_BYTES_PER_PHYSICAL_PAGE + NAND_FLASH_SPARE_BYTES_PER_PAGE] = {0};

    if (main_buffer) {
        memcpy(buffer, main_buffer, NAND_FLASH_BYTES_PER_PHYSICAL_PAGE);
    } else {
        memset(buffer, 0xFF, NAND_FLASH_BYTES_PER_PHYSICAL_PAGE);
    }

    if (spare_buffer) {
        memcpy(buffer + NAND_FLASH_BYTES_PER_PHYSICAL_PAGE, spare_buffer, NAND_FLASH_SPARE_BYTES_PER_PAGE);
    } else {
        memset(buffer + NAND_FLASH_BYTES_PER_PHYSICAL_PAGE, 0xFF, NAND_FLASH_SPARE_BYTES_PER_PAGE);
    }

    // Wait in case the previous operation has not completed.
    if (nand_flash_wait_until_ready(hospi) != 0)
        return -1;

    if (nand_flash_enable_write(hospi) != 0)
        return -1;

    if (nand_flash_command_program_load(hospi, 0, buffer, (uint16_t)(NAND_FLASH_BYTES_PER_PHYSICAL_PAGE + NAND_FLASH_SPARE_BYTES_PER_PAGE)) != 0)
        return -1;

    if (nand_flash_command_program_execute(hospi, row_address) != 0)
        return -1;

    // Wait for the current operation to finish.
    if (nand_flash_wait_until_ready(hospi) != 0)
        return -1;

    if (nand_flash_check_program_failure(hospi) != 0)
        return -1;

    return 0;
}

int nand_flash_pages_read(OSPI_HandleTypeDef *hospi, uint32_t block, uint32_t page, uint8_t *main_buffer, uint8_t *spare_buffer, uint32_t pages) {
    // Wait in case the previous operation has not completed yet.
    if (nand_flash_wait_until_ready(hospi) != 0)
        return -1;

    for (uint32_t index = 0; index < pages; index++) {
        uint32_t row_address;

        if (nand_flash_calculate_row_address(block, page + index, &row_address) != 0)
            return -1;

        if (nand_flash_command_page_read_to_cache(hospi, row_address) != 0)
            return -1;

        // Wait for the current operation to finish.
        if (nand_flash_wait_until_ready(hospi) != 0)
            return -1;

        if (nand_flash_check_ecc_failure(hospi) != 0)
            return -1;

        if (main_buffer) {
            if (nand_flash_command_read_from_cache_x1(hospi, 0, main_buffer + (index * NAND_FLASH_BYTES_PER_PHYSICAL_PAGE), NAND_FLASH_BYTES_PER_PHYSICAL_PAGE) != 0)
                return -1;
        }

        if (spare_buffer) {
            if (nand_flash_command_read_from_cache_x1(hospi, NAND_FLASH_BYTES_PER_PHYSICAL_PAGE, spare_buffer + (index * NAND_FLASH_SPARE_BYTES_PER_PAGE), NAND_FLASH_SPARE_BYTES_PER_PAGE) != 0)
                return -1;
        }
    }

    return 0;
}

int nand_flash_pages_write(OSPI_HandleTypeDef *hospi, uint32_t block, uint32_t page, uint8_t *main_buffer, uint8_t *spare_buffer, uint32_t pages) {
    for (uint32_t index = 0; index < pages; index++) {
        uint32_t row_address;

        if (nand_flash_calculate_row_address(block, page + index, &row_address) != 0)
            return -1;

        if (nand_flash_program_page(hospi,
                                    row_address,
                                    main_buffer ? main_buffer + (index * NAND_FLASH_BYTES_PER_PHYSICAL_PAGE) : NULL,
                                    spare_buffer ? spare_buffer + (index * NAND_FLASH_SPARE_BYTES_PER_PAGE) : NULL) != 0)
            return -1;
    }

    return 0;
}

int nand_flash_pages_copy(OSPI_HandleTypeDef *hospi, uint32_t source_block, uint32_t source_page, uint32_t destination_block, uint32_t destination_page, uint32_t pages, uint8_t *data_buffer) {
    // Wait in case the previous operation has not completed yet.
    if (nand_flash_wait_until_ready(hospi) != 0)
        return -1;

    for (uint32_t index = 0; index < pages; index++) {
        uint32_t source_row_address;
        uint32_t destination_row_address;

        if (nand_flash_calculate_row_address(source_block, source_page + index, &source_row_address) != 0)
            return -1;

        if (nand_flash_calculate_row_address(destination_block, destination_page + index, &destination_row_address) != 0)
            return -1;

        if (nand_flash_command_page_read_to_cache(hospi, source_row_address) != 0)
            return -1;

        // Wait for the current read operation to finish.
        if (nand_flash_wait_until_ready(hospi) != 0)
            return -1;

        if (nand_flash_check_ecc_failure(hospi) != 0)
            return -1;

        if (nand_flash_command_read_from_cache_x1(hospi, 0, data_buffer, NAND_FLASH_BYTES_PER_PHYSICAL_PAGE + NAND_FLASH_SPARE_BYTES_PER_PAGE) != 0)
            return -1;

        if (nand_flash_enable_write(hospi) != 0)
            return -1;

        if (nand_flash_command_program_load(hospi, 0, data_buffer, NAND_FLASH_BYTES_PER_PHYSICAL_PAGE + NAND_FLASH_SPARE_BYTES_PER_PAGE) != 0)
            return -1;

        if (nand_flash_command_program_execute(hospi, destination_row_address) != 0)
            return -1;

        // Wait for the current program operatoin to finish.
        if (nand_flash_wait_until_ready(hospi) != 0)
            return -1;

        if (nand_flash_check_program_failure(hospi) != 0)
            return -1;
    }

    return 0;
}

int nand_flash_block_erase(OSPI_HandleTypeDef *hospi, uint32_t block, uint32_t erase_count) {
    (void)erase_count;

    // Wait in case the previous operation has not completed yet.
    if (nand_flash_wait_until_ready(hospi) != 0)
        return -1;

    uint32_t row_address;

    if (nand_flash_calculate_row_address(block, 0, &row_address) != 0)
        return -1;

    if (nand_flash_enable_write(hospi) != 0)
        return -1;

    if (nand_flash_command_block_erase(hospi, row_address) != 0)
        return -1;

    if (nand_flash_wait_until_ready(hospi) != 0)
        return -1;

    if (nand_flash_check_erase_failure(hospi) != 0)
        return -1;

    return 0;
}

int nand_flash_page_erased_verify(OSPI_HandleTypeDef *hospi, uint32_t block, uint32_t page) {
    uint8_t main_buffer[NAND_FLASH_BYTES_PER_PHYSICAL_PAGE];
    uint8_t spare_buffer[NAND_FLASH_SPARE_BYTES_PER_PAGE];

    if (nand_flash_pages_read(hospi, block, page, main_buffer, spare_buffer, 1) != 0)
        return -1;

    for (uint32_t index = 0; index < NAND_FLASH_BYTES_PER_PHYSICAL_PAGE; index++) {
        if (main_buffer[index] != 0xFF)
            return -1;
    }

    for (uint32_t index = 0; index < NAND_FLASH_SPARE_BYTES_PER_PAGE; index++) {
        if (spare_buffer[index] != 0xFF)
            return -1;
    }

    return 0;
}

int nand_flash_block_erased_verify(OSPI_HandleTypeDef *hospi, uint32_t block) {
    for (uint32_t page = 0; page < NAND_FLASH_PAGES_PER_BLOCK; page++) {
        if (nand_flash_page_erased_verify(hospi, block, page) != 0)
            return -1;
    }

    return 0;
}

int nand_flash_block_status_get(OSPI_HandleTypeDef *hospi, uint32_t block, uint8_t *bad_block_byte) {
    uint8_t spare_buffer[NAND_FLASH_SPARE_BYTES_PER_PAGE];

    if (!bad_block_byte)
        return -1;

    if (nand_flash_pages_read(hospi, block, 0, NULL, spare_buffer, 1) != 0)
        return -1;

    *bad_block_byte = spare_buffer[NAND_FLASH_BAD_BLOCK_POSITION];

    return 0;
}

int nand_flash_block_status_set(OSPI_HandleTypeDef *hospi, uint32_t block, uint8_t bad_block_byte) {
    uint8_t spare_buffer[NAND_FLASH_SPARE_BYTES_PER_PAGE];

    memset(spare_buffer, 0xFF, sizeof(spare_buffer));
    spare_buffer[NAND_FLASH_BAD_BLOCK_POSITION] = bad_block_byte;

    uint32_t row_address;

    if (nand_flash_calculate_row_address(block, 0, &row_address) != 0)
        return -1;

    if (nand_flash_program_page(hospi, row_address, NULL, spare_buffer) != 0)
        return -1;

    return 0;
}