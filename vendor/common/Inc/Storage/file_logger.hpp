#pragma once
#include "fx_api.h"
#include "stm32h723xx.h"
#include "storage_manager.hpp"
#include "timebase.hpp"
#include "tx_api.h"
#include <cstdint>

struct __attribute__((packed)) LogEntryHeader {
  uint16_t magic        = 0; // Sync word (logger_magic_number), set on enqueue().
  uint16_t checksum     = 0;
  uint16_t id           = 0;
  uint16_t size         = 0;
  uint64_t reserved0    = 0; // Currently unused. Must always be zero. Reserved for future features.
  uint64_t timestamp_us = 0;
};

struct LogEntry {
  LogEntryHeader header;
  uint8_t *data;
};

enum class FileLoggerState : uint8_t {
  NotStarted = 0,
  WaitingForStorage, // Storage media is not ready yet.
  CreateError,       // fx_file_create failed.
  OpenError,         // fx_file_open failed.
  SeekError,         // fx_file_relative_seek failed.
  Running,           // File is open; accepting and writing entries.
  WriteError,        // fx_file_write failed.
};

#pragma pack(push, 1)
struct FileLoggerTelemetry {
  uint32_t write_bps;        // Windowed fx_file_write throughput, bytes per second.
  uint32_t write_us_max;     // Worst fx_file_write duration (us) in the last sampling window.
  uint32_t enqueue_accepted; // Entries that made it onto the queue in the last window.
  uint32_t enqueue_rejected; // enqueue() calls rejected in the last window.
  FileLoggerState state;     // Current logger state.
};
#pragma pack(pop)

class FileLogger {
  static constexpr uint16_t logger_magic_number = 0xBEEF;

  static constexpr ULONG logger_ready_flag = (1 << 0);

  static constexpr size_t logger_queue_entry_count          = 1024;      // Count, not bytes!
  static constexpr size_t logger_pool_buffer_size           = 32 * 1024; // 64 kB
  static constexpr size_t telemetry_file_buffer_size        = 32 * 1024; // 64 kB
  static constexpr size_t telemetry_file_buffer_ullage_size = 1 * 1024;  // 1 kB

  static constexpr ULONG poll_interval_ticks = TX_TIMER_TICKS_PER_SECOND / 10; // 100 ms
  static constexpr uint64_t throughput_sample_interval_us = 1000000; // 1 s

  char logger_name[32]   = {};
  char log_file_name[32] = {};

  TX_QUEUE logger_queue                                                    = {};
  uint8_t logger_queue_buffer[sizeof(LogEntry) * logger_queue_entry_count] = {};

  TX_BYTE_POOL logger_pool                            = {};
  uint8_t logger_pool_buffer[logger_pool_buffer_size] = {};

  TX_EVENT_FLAGS_GROUP logger_flags = {};

  StorageManager &storage_manager;

  FX_FILE telemetry_file                                                   = {};
  ALIGN_32BYTES(uint8_t telemetry_file_buffer[telemetry_file_buffer_size]) = {};

  GPIO_TypeDef *const led_port = nullptr;
  const uint16_t led_pin       = 0;

  uint32_t write_bps        = 0;
  uint32_t write_us_max     = 0;
  uint32_t enqueue_accepted = 0;
  uint32_t enqueue_rejected = 0;

  uint64_t speed_last_us          = 0;
  uint32_t write_bytes_accum      = 0;
  uint32_t write_us_max_accum     = 0;
  uint32_t enqueue_accepted_accum = 0;
  uint32_t enqueue_rejected_accum = 0;

  FileLoggerState state = FileLoggerState::NotStarted;

public:
  FileLogger(StorageManager &storage_manager, const char *logger_name, const char *log_file_name,
             GPIO_TypeDef *const led_port = nullptr, const uint16_t led_pin = 0);

  UINT init(void);

  UINT enqueue(const uint16_t id, const uint16_t data_size, const uint8_t *const data);

  FileLoggerTelemetry get_telemetry(void);

private:
  UINT dequeue(uint8_t *const dest, uint16_t dest_size, uint16_t *const entry_size);

  void reset_throughput(void);
  void update_throughput(uint32_t bytes, uint32_t write_us);

  void led_on(void);
  void led_off(void);

public:
  [[noreturn]] void run(void);
};
