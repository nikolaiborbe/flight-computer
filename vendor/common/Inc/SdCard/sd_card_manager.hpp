#pragma once
#include "fx_api.h"
#include "fx_propulse_sd_driver.h"
#include "stm32h7xx_hal.h"
#include "storage_manager.hpp"
#include "tx_api.h"

class SdCardManager : public StorageManager {
  SD_HandleTypeDef *hsd;

  const GPIO_TypeDef *card_detect_port;
  const uint16_t card_detect_pin;

  static constexpr size_t secor_size = 512;
  ALIGN_32BYTES(uint32_t media_memory[16 * secor_size / sizeof(uint32_t)]);

public:
  SdCardManager(SD_HandleTypeDef *hsd, const GPIO_TypeDef *card_detect_port,
                const uint16_t card_detect_pin, const char *manager_name, const char *volume_name)
      : StorageManager(manager_name, volume_name), hsd(hsd), card_detect_port(card_detect_port),
        card_detect_pin(card_detect_pin) {}

private:
  UINT device_detected() override;

  UINT init_peripheral() override;

  UINT format() override;

  UINT open() override;
};
