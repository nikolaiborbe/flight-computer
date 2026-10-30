#pragma once
#include "fx_api.h"
#include "stm32h7xx_hal.h"
#include "tx_api.h"
#include <cstdint>
#include <cstring>

enum class StorageState : uint8_t {
  NotStarted = 0,
  NoDevice,
  Formatting,
  Running,
  PeripheralError, // Peripheral init or a flush failed.
  FormattingError,
  OpenError, // Peripheral is up but the volume would not mount.
};

// Packed for a stable telemetry wire layout.
#pragma pack(push, 1)
struct StorageTelemetry {
  uint64_t total_bytes;
  uint64_t free_bytes;
  uint32_t read_bps; // Bytes per second.
  uint32_t write_bps;
  StorageState state;
  uint8_t device_present;
};
#pragma pack(pop)

class StorageManager {
  static constexpr ULONG storage_ready_flag    = (1 << 0);
  static constexpr ULONG format_requested_flag = (1 << 1);

  // Derived from the tick rate so the timings hold at any RTOS tick frequency.
  static constexpr ULONG poll_interval_ticks = TX_TIMER_TICKS_PER_SECOND / 2;         // 500 ms
  static constexpr ULONG throughput_sample_interval_ticks = TX_TIMER_TICKS_PER_SECOND; // 1 s

  char manager_name[32]      = {};
  TX_EVENT_FLAGS_GROUP flags = {};

  // Written by the worker thread and read by the telemetry thread. Single-word
  // access is atomic, so an occasional stale read is harmless.
  StorageState state = StorageState::NotStarted;

  // Latest throughput estimate in bytes per second.
  uint32_t read_bps  = 0;
  uint32_t write_bps = 0;

  // Baseline for the periodic throughput sampling.
  ULONG speed_last_ticks = 0;
  ULONG speed_last_read  = 0;
  ULONG speed_last_write = 0;

protected:
  char volume_name[32] = {};
  FX_MEDIA media       = {};

public:
  StorageManager(const char *manager_name, const char *volume_name);

  UINT init();

  UINT get_media(FX_MEDIA **out_media, UINT get_option = TX_WAIT_FOREVER);

  UINT is_ready();

  UINT request_format();

  StorageTelemetry get_telemetry();

private:
  virtual UINT device_detected() = 0;

  virtual UINT init_peripheral() = 0;

  virtual UINT format() = 0;

  virtual UINT open() = 0;

  void reset_throughput();

  void update_throughput();

public:
  [[noreturn]] void run();
};
