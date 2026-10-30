#pragma once

#include <cstdint>

// IMPORTANT: Before using this make sure that the hard coded
// flash address and size match the actual flash memory of the
// STM32 MCU used!!!

class BootCounter {
public:
  BootCounter();

  void increment();
  uint32_t get();

private:
  struct Entry {
    uint32_t magic;
    uint32_t counter;
    uint32_t counter_inverted;
    uint8_t reserved[20];
  };

  static constexpr uint32_t magic = 0xB007C0DE;

  // Needs to be reserved in linker script!!!
  // Last 128 KB flash sector of STM32H723VGT.
  static constexpr uint32_t flash_address = 0x080E0000;
  static constexpr uint32_t flash_size    = 128 * 1024;

  static_assert(sizeof(Entry) == 32, "Boot counter entry must be exactly one flashword");

  static_assert((flash_address % 32) == 0, "Boot counter address must be 32-byte aligned");

  uint32_t counter = 0;

  bool initialized = false;
  bool incremented = false;

  void init();

  uint32_t find_counter();

  bool try_write_counter(uint32_t counter);
  bool write_entry(uint32_t address, uint32_t counter);

  bool erase_page();

  bool is_valid(const volatile Entry &entry);
  bool is_erased(const volatile Entry &entry);
};
