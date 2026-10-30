#include "boot_counter.hpp"
#include "stm32h7xx_hal.h"

BootCounter::BootCounter() {}

bool BootCounter::is_valid(const volatile Entry &entry) {
  return entry.magic == magic && entry.counter_inverted == ~entry.counter;
}

bool BootCounter::is_erased(const volatile Entry &entry) {
  const volatile uint8_t *data = reinterpret_cast<const volatile uint8_t *>(&entry);

  for (uint32_t i = 0; i < sizeof(Entry); i++) {
    if (data[i] != 0xFF)
      return false;
  }

  return true;
}

void BootCounter::init() {
  if (initialized)
    return;

  counter = find_counter();

  initialized = true;
}

uint32_t BootCounter::get() {
  init();

  return counter;
}

void BootCounter::increment() {
  init();

  // Only ever increment once per boot.
  if (incremented)
    return;

  uint32_t next_counter = counter + 1;

  if (try_write_counter(next_counter)) {
    counter     = next_counter;
    incremented = true;
  }
}

uint32_t BootCounter::find_counter() {
  uint32_t highest_counter = 0;

  for (uint32_t address = flash_address; address < flash_address + flash_size;
       address += sizeof(Entry)) {

    const volatile Entry &entry = *reinterpret_cast<const volatile Entry *>(address);

    // Ignore invalid entries.
    if (!is_valid(reinterpret_cast<const volatile Entry &>(entry)))
      continue;

    if (entry.counter > highest_counter)
      highest_counter = entry.counter;
  }

  return highest_counter;
}

bool BootCounter::try_write_counter(uint32_t value) {
  // Try every available slot before giving up.
  for (uint32_t address = flash_address; address < flash_address + flash_size;
       address += sizeof(Entry)) {

    const volatile Entry &entry = *reinterpret_cast<const volatile Entry *>(address);

    if (!is_erased(reinterpret_cast<const volatile Entry &>(entry)))
      continue;

    if (write_entry(address, value))
      return true;
  }

  // No usable slots left, erase sector.
  if (!erase_page())
    return false;

  // Try first slot after erase.
  return write_entry(flash_address, value);
}
bool BootCounter::write_entry(uint32_t address, uint32_t value) {
  Entry entry{};

  entry.magic            = magic;
  entry.counter          = value;
  entry.counter_inverted = ~value;

  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  HAL_FLASH_Unlock();

  HAL_StatusTypeDef status =
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, reinterpret_cast<uint32_t>(&entry));

  HAL_FLASH_Lock();

  __set_PRIMASK(primask);

  if (status != HAL_OK)
    return false;

  // Flash contents changed, invalidate cached data before verification.
  SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t *>(address), sizeof(Entry));

  // Verify written data
  const volatile Entry &written = *reinterpret_cast<const volatile Entry *>(address);

  return is_valid(written) && written.counter == value && written.counter_inverted == ~value;
}

bool BootCounter::erase_page() {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef erase{};
  uint32_t error = 0;

  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Banks     = FLASH_BANK_1;
  erase.Sector    = FLASH_SECTOR_7;
  erase.NbSectors = 1;

  HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &error);

  HAL_FLASH_Lock();

  __set_PRIMASK(primask);

  if (status != HAL_OK)
    return false;

  // Flash contents changed, invalidate cached data before verification.
  SCB_InvalidateDCache_by_Addr(reinterpret_cast<uint32_t *>(flash_address), flash_size);

  // Verify that the entire sector was erased.
  for (uint32_t address = flash_address; address < flash_address + flash_size;
       address += sizeof(Entry)) {

    const volatile Entry &entry = *reinterpret_cast<const volatile Entry *>(address);

    if (!is_erased(entry))
      return false;
  }

  return true;
}
