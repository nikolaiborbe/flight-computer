#include "nand_flash_test.h"

int nand_flash_test(OSPI_HandleTypeDef *hospi) {
    uint8_t feature_otp, feature_status;
    
    ///////////////////
    // Test 0: Reset //
    ///////////////////

    // Note: We don't currently test if the reset itself actually worked.

    if (nand_flash_command_reset(hospi) < 0)
        return -00;

    if (nand_flash_wait_until_ready(hospi) < 0)
        return -01;

    if (nand_flash_command_set_feature(hospi, FLASH_FEATURE_ADDRESS_BLOCK_LOCK, 0) < 0)
        return -02;

    /////////////////////
    // Test 1: Read ID //
    /////////////////////

    uint8_t manufacturer_id;
    uint8_t device_id;

    if (nand_flash_command_read_id(hospi, &manufacturer_id, &device_id) < 0)
        return -10;

    if (manufacturer_id != FLASH_MANUFACTURER_ID)
        return -11;

    if (device_id != FLASH_DEVICE_ID_32 && device_id != FLASH_DEVICE_ID_34)
        return -12;

    //////////////////////////
    // Test 2: Write Enable //
    //////////////////////////

    if (nand_flash_command_write_enable(hospi) < 0)
        return -20;

    if (nand_flash_command_get_feature(hospi, FLASH_FEATURE_ADDRESS_STATUS, &feature_status) < 0)
        return -21;
    
    if (!(feature_status & FLASH_FEATURE_STATUS_WEL))
        return -22;
    
    ///////////////////////////
    // Test 3: Write Disable //
    ///////////////////////////
    
    if (nand_flash_command_write_disable(hospi) < 0)
        return -30;

    if (nand_flash_command_get_feature(hospi, FLASH_FEATURE_ADDRESS_STATUS, &feature_status) < 0)
        return -31;
    
    if (feature_status & FLASH_FEATURE_STATUS_WEL)
        return -32;

    ///////////////////////////////
    // Test 4: Feature Set & Get //
    ///////////////////////////////

    // Disable ECC and confirm that it is in fact disabled.

    if (nand_flash_command_set_feature(hospi, FLASH_FEATURE_ADDRESS_OTP, 0) < 0)
        return -40;

    if (nand_flash_command_get_feature(hospi, FLASH_FEATURE_ADDRESS_OTP, &feature_otp) < 0)
        return -41;

    if (feature_otp & FLASH_FEATURE_OTP_ECC_EN)
        return -42;

    // Enable ECC and confirm that it is in fact enabled.
    
    if (nand_flash_command_set_feature(hospi, FLASH_FEATURE_ADDRESS_OTP, FLASH_FEATURE_OTP_ECC_EN) < 0)
        return -43;

    if (nand_flash_command_get_feature(hospi, FLASH_FEATURE_ADDRESS_OTP, &feature_otp) < 0)
        return -44;

    if (!(feature_otp & FLASH_FEATURE_OTP_ECC_EN))
        return -45;

    ////////////////////////////////
    // Test 5: Erase & Read Block //
    ////////////////////////////////

    if (nand_flash_block_erase(hospi, 0, 0) < 0)
        return -50;

    if (nand_flash_block_erased_verify(hospi, 0) < 0)
        return -51;

    ////////////////////////////////
    // Test 6: Write & Read Block //
    ////////////////////////////////
    
    uint8_t write_buffer[NAND_FLASH_BYTES_PER_PHYSICAL_PAGE] = "Paulius";
    uint8_t read_buffer[NAND_FLASH_BYTES_PER_PHYSICAL_PAGE];

    if (nand_flash_pages_write(hospi, 0, 0, write_buffer, NULL, 1) < 0)
        return -60;

    if (nand_flash_pages_read(hospi, 0, 0, read_buffer, NULL, 1) < 0)
        return -61;

    for (int i = 0; i < NAND_FLASH_BYTES_PER_PHYSICAL_PAGE; i++) {
        if (write_buffer[i] != read_buffer[i])
            return -62;
    }

    return 0;
}