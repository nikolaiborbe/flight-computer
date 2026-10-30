#include "sd_card_manager.hpp"

UINT SdCardManager::device_detected() {
  return HAL_GPIO_ReadPin(card_detect_port, card_detect_pin) == GPIO_PIN_RESET;
}

UINT SdCardManager::init_peripheral() {
  if (!device_detected())
    return HAL_ERROR;

  UINT status;

  status = HAL_SD_DeInit(hsd);
  if (status != HAL_OK)
    return status;

  __HAL_RCC_SDMMC1_FORCE_RESET();
  __NOP();
  __HAL_RCC_SDMMC1_RELEASE_RESET();

  hsd->Instance                 = SDMMC1;
  hsd->Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
  hsd->Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd->Init.BusWide             = SDMMC_BUS_WIDE_4B;
  hsd->Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd->Init.ClockDiv            = 1;

  status = HAL_SD_Init(hsd);
  if (status != HAL_OK) {
    HAL_SD_DeInit(hsd);
    return status;
  }

  return HAL_OK;
}

UINT SdCardManager::format() {
  HAL_SD_CardInfoTypeDef card_info;

  UINT status = HAL_SD_GetCardInfo(hsd, &card_info);

  if (status != HAL_OK)
    return status;

  return fx_media_format(&media,                // SD_Disk pointer
                         fx_propulse_sd_driver,    // Driver entry
                         (VOID *)FX_NULL,       // Device info pointer
                         (UCHAR *)media_memory, // Media buffer pointer
                         sizeof(media_memory),  // Media buffer size
                         volume_name,           // Volume Name
                         1,                     // Number of FATs
                         32,                    // Directory Entries
                         0,                     // Hidden sectors
                         card_info.BlockNbr,    // Total sectors
                         512,                   // Sector size
                         8,                     // Sectors per cluster
                         1,                     // Heads
                         1);                    // Sectors per track
}

UINT SdCardManager::open() {
  return fx_media_open(&media, volume_name, fx_propulse_sd_driver, (VOID *)FX_NULL,
                       (VOID *)media_memory, sizeof(media_memory));
}