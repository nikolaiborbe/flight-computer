/**
 * @file flash.c
 * @author Paulius Juzenas (paulius.juzenas@propulse.no)
 * @brief Simple driver for the AS5F34G04SNDB-08LIN flash chip.
 * @version 0.1
 * @date 2026-06-08
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "nand_flash_commands.h"

#define TIMEOUT_TICKS (1000)

int nand_flash_command_write_enable(OSPI_HandleTypeDef *hospi) {
    OSPI_RegularCmdTypeDef cmd = {0};

    cmd.Instruction        = 0x06;
    cmd.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;

    if (HAL_OSPI_Command(hospi, &cmd, TIMEOUT_TICKS) != HAL_OK)
        return -1;

    return 0;
}

int nand_flash_command_write_disable(OSPI_HandleTypeDef *hospi) {
    OSPI_RegularCmdTypeDef cmd = {0};

    cmd.Instruction        = 0x04;
    cmd.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;

    if (HAL_OSPI_Command(hospi, &cmd, TIMEOUT_TICKS) != HAL_OK)
        return -1;

    return 0;
}

int nand_flash_command_get_feature(OSPI_HandleTypeDef *hospi, uint8_t feature_address, uint8_t *out_feature) {
    OSPI_RegularCmdTypeDef cmd = {0};

    cmd.Instruction        = 0x0F;
    cmd.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;

    cmd.Address            = feature_address;
    cmd.AddressMode        = HAL_OSPI_ADDRESS_1_LINE;

    cmd.DataMode           = HAL_OSPI_DATA_1_LINE;
    cmd.NbData             = 1;

    uint8_t feature;

    if (HAL_OSPI_Command(hospi, &cmd, TIMEOUT_TICKS) != HAL_OK)
        return -1;

    if (HAL_OSPI_Receive(hospi, &feature, TIMEOUT_TICKS) != HAL_OK)
        return -1;

    if (out_feature)
        *out_feature = feature;

    return 0;
}

int nand_flash_command_set_feature(OSPI_HandleTypeDef *hospi, uint8_t feature_address, uint8_t feature) {
    OSPI_RegularCmdTypeDef cmd = {0};

    cmd.Instruction        = 0x1F;
    cmd.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;

    cmd.Address            = feature_address;
    cmd.AddressMode        = HAL_OSPI_ADDRESS_1_LINE;

    cmd.DataMode           = HAL_OSPI_DATA_1_LINE;
    cmd.NbData             = 1;

    if (HAL_OSPI_Command(hospi, &cmd, TIMEOUT_TICKS) != HAL_OK)
        return -1;

    if (HAL_OSPI_Transmit(hospi, &feature, TIMEOUT_TICKS) != HAL_OK)
        return -1;

    return 0;
};

int nand_flash_command_read_id(OSPI_HandleTypeDef *hospi, uint8_t *out_manufacturer_id, uint8_t *out_device_id) {
    OSPI_RegularCmdTypeDef cmd = {0};

    cmd.Instruction        = 0x9F;
    cmd.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;

    cmd.Address            = 0x00;
    cmd.AddressMode        = HAL_OSPI_ADDRESS_1_LINE;

    cmd.DataMode           = HAL_OSPI_DATA_1_LINE;
    cmd.NbData             = 2;

    uint8_t ids[2] = {0};

    if (HAL_OSPI_Command(hospi, &cmd, TIMEOUT_TICKS) != HAL_OK)
        return -1;

    if (HAL_OSPI_Receive(hospi, ids, TIMEOUT_TICKS) != HAL_OK)
        return -1;

    if (out_manufacturer_id)
        *out_manufacturer_id = ids[0];

    if (out_device_id)
        *out_device_id = ids[1];

    return 0;
}

int nand_flash_command_page_read_to_cache(OSPI_HandleTypeDef *hospi, uint32_t row_address) {
    OSPI_RegularCmdTypeDef cmd = {0};

    cmd.Instruction        = 0x13;
    cmd.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;

    cmd.Address            = row_address;
    cmd.AddressMode        = HAL_OSPI_ADDRESS_1_LINE;
    cmd.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;;

    if (HAL_OSPI_Command(hospi, &cmd, TIMEOUT_TICKS) != HAL_OK)
        return -1;

    return 0;
}

int nand_flash_command_read_from_cache_x1(OSPI_HandleTypeDef *hospi, uint16_t column_address, uint8_t *out_data, uint16_t out_data_length) {
    if (!out_data)
        return -2;

    OSPI_RegularCmdTypeDef cmd = {0};

    cmd.Instruction        = 0x03;
    cmd.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;

    cmd.Address            = column_address;
    cmd.AddressMode        = HAL_OSPI_ADDRESS_1_LINE;
    cmd.AddressSize        = HAL_OSPI_ADDRESS_16_BITS;

    cmd.DummyCycles = 8;

    cmd.DataMode = HAL_OSPI_DATA_1_LINE;
    cmd.NbData = out_data_length;

    if (HAL_OSPI_Command(hospi, &cmd, TIMEOUT_TICKS) != HAL_OK)
        return -1;

    if (HAL_OSPI_Receive(hospi, out_data, TIMEOUT_TICKS) != HAL_OK)
        return -1;
    
    return 0;
}

int nand_flash_command_program_load(OSPI_HandleTypeDef *hospi, uint16_t column_address, uint8_t *data, uint16_t data_length) {
    if (!data)
        return -2;

    OSPI_RegularCmdTypeDef cmd = {0};

    cmd.Instruction        = 0x02;
    cmd.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;

    cmd.Address            = column_address;
    cmd.AddressMode        = HAL_OSPI_ADDRESS_1_LINE;
    cmd.AddressSize        = HAL_OSPI_ADDRESS_16_BITS;

    cmd.DataMode = HAL_OSPI_DATA_1_LINE;
    cmd.NbData = data_length;

    if (HAL_OSPI_Command(hospi, &cmd, TIMEOUT_TICKS) != HAL_OK)
        return -1;

    if (HAL_OSPI_Transmit(hospi, data, TIMEOUT_TICKS) != HAL_OK)
        return -1;
    
    return 0;
}

int nand_flash_command_program_execute(OSPI_HandleTypeDef *hospi, uint32_t row_address) {
    OSPI_RegularCmdTypeDef cmd = {0};

    cmd.Instruction        = 0x10;
    cmd.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;

    cmd.Address            = row_address;
    cmd.AddressMode        = HAL_OSPI_ADDRESS_1_LINE;
    cmd.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;;

    if (HAL_OSPI_Command(hospi, &cmd, TIMEOUT_TICKS) != HAL_OK)
        return -1;

    return 0;
}

int nand_flash_command_block_erase(OSPI_HandleTypeDef *hospi, uint32_t row_address) {
    OSPI_RegularCmdTypeDef cmd = {0};

    cmd.Instruction        = 0xD8;
    cmd.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;

    cmd.Address            = row_address;
    cmd.AddressMode        = HAL_OSPI_ADDRESS_1_LINE;
    cmd.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;;

    if (HAL_OSPI_Command(hospi, &cmd, TIMEOUT_TICKS) != HAL_OK)
        return -1;
    
    return 0;
}

int nand_flash_command_reset(OSPI_HandleTypeDef *hospi) {
    OSPI_RegularCmdTypeDef cmd = {0};

    cmd.Instruction        = 0xFF;
    cmd.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;

    if (HAL_OSPI_Command(hospi, &cmd, TIMEOUT_TICKS) != HAL_OK)
        return -1;

    return 0;
}
