#include "storage_manager.hpp"

StorageManager::StorageManager(const char *manager_name, const char *volume_name) {
  strncpy(this->manager_name, manager_name, sizeof(this->manager_name) - 1);
  this->manager_name[sizeof(this->manager_name) - 1] = '\0';
  strncpy(this->volume_name, volume_name, sizeof(this->volume_name) - 1);
  this->volume_name[sizeof(this->volume_name) - 1] = '\0';
};

UINT StorageManager::init() { return tx_event_flags_create(&flags, manager_name); };

UINT StorageManager::get_media(FX_MEDIA **out_media, UINT get_option) {
  UINT status;
  ULONG actual_flags;

  status = tx_event_flags_get(&flags, storage_ready_flag, TX_AND, &actual_flags, get_option);
  if (status != TX_SUCCESS)
    return status;

  *out_media = &media;

  return TX_SUCCESS;
}

UINT StorageManager::is_ready() {
  UINT status;
  ULONG actual_flags;

  status = tx_event_flags_get(&flags, storage_ready_flag, TX_AND, &actual_flags, TX_NO_WAIT);
  if (status != TX_SUCCESS)
    return status;

  return TX_SUCCESS;
}

UINT StorageManager::request_format() {
  return tx_event_flags_set(&flags, format_requested_flag, TX_OR);
}

StorageTelemetry StorageManager::get_telemetry() {
  StorageTelemetry t{};
  t.state = state;
  t.device_present = device_detected() ? 1 : 0;

  // Space and throughput only make sense while mounted. 
  // The media fields are read without locking. This is
  // fine since all the statistics are simply ULONGS.
  if (state == StorageState::Running) {
    const UINT bytes_per_sector = media.fx_media_bytes_per_sector;
    t.total_bytes = (uint64_t)media.fx_media_total_sectors * bytes_per_sector;
    t.free_bytes = (uint64_t)media.fx_media_available_clusters *
                   media.fx_media_sectors_per_cluster * bytes_per_sector;
    t.read_bps = read_bps;
    t.write_bps = write_bps;
  }

  return t;
}

void StorageManager::reset_throughput() {
  speed_last_ticks = tx_time_get();
  speed_last_read = media.fx_media_logical_sector_reads;
  speed_last_write = media.fx_media_logical_sector_writes;
  read_bps = write_bps = 0;
}

void StorageManager::update_throughput() {
  // Calculate how many ticks have passed since the previous call.
  const ULONG now_ticks = tx_time_get();
  const ULONG elapsed_ticks = now_ticks - speed_last_ticks;
  
  // Skip calculations if the time since the previous call is
  // less than the throughput calculation interval.
  if (elapsed_ticks < throughput_sample_interval_ticks)
    return;

  // Retrieve statistics from the FileX media struct.
  // This can be safely done without locking as we are simply
  // reading individual ULONGs.
  // Note that if the MEDIA object is recreated then sectors
  // read/written is reset to zero, so we need to account for that
  // when calculating the read/write delta.
  const ULONG bytes_per_sector = media.fx_media_bytes_per_sector;
  const ULONG read_sectors = media.fx_media_logical_sector_reads;
  const ULONG write_sectors = media.fx_media_logical_sector_writes;
  const ULONG read_sector_delta =
      (read_sectors >= speed_last_read) ? (read_sectors - speed_last_read) : 0;
  const ULONG write_sector_delta =
      (write_sectors >= speed_last_write) ? (write_sectors - speed_last_write) : 0;

  // A 64-bit intermediate keeps the sector-to-byte scaling from overflowing.
  read_bps = (uint32_t)((uint64_t)read_sector_delta * bytes_per_sector *
                        TX_TIMER_TICKS_PER_SECOND / elapsed_ticks);
  write_bps = (uint32_t)((uint64_t)write_sector_delta * bytes_per_sector *
                         TX_TIMER_TICKS_PER_SECOND / elapsed_ticks);

  speed_last_read = read_sectors;
  speed_last_write = write_sectors;
  speed_last_ticks = now_ticks;
}

void StorageManager::run() {
  ULONG actual_flags;

  while (true) {
    tx_thread_sleep(poll_interval_ticks);

    if (!device_detected()) {
      state = StorageState::NoDevice;
      continue;
    }


    // Re-init the storage peripheral to be in a known state.
    if (init_peripheral() != HAL_OK) {
      state = StorageState::PeripheralError;
      continue;
    }

    // Check if format has been requested. The request flag is only cleared
    // once we've actually attempted the format (below), not on read.
    if (tx_event_flags_get(&flags, format_requested_flag, TX_AND, &actual_flags, TX_NO_WAIT) == TX_SUCCESS) {
      state = StorageState::Formatting;
      if (format() != FX_SUCCESS) {
        state = StorageState::FormattingError;
        tx_event_flags_set(&flags, ~format_requested_flag, TX_AND);
        continue;
      }
      tx_event_flags_set(&flags, ~format_requested_flag, TX_AND);
    }

    // Try to open the volume on the media.
    // If this fails the safest thing is to do nothing.
    if (open() != FX_SUCCESS) {
      state = StorageState::OpenError;
      continue;
    }

    tx_event_flags_set(&flags, storage_ready_flag, TX_OR);
    state = StorageState::Running;
    reset_throughput();

    while (true) {
      // If the device has been removed, restart everything.
      if (!device_detected()) {
        state = StorageState::NoDevice;
        break;
      }

      // A format was requested. No clean close is needed as the media is aborted
      // below and a reformat discards the data anyway.
      if (tx_event_flags_get(&flags, format_requested_flag, TX_AND, &actual_flags, TX_NO_WAIT) ==
          TX_SUCCESS) {
        state = StorageState::Formatting;
        break;
      }

      update_throughput();

      // Flush periodically to persist buffered data. A failure usually means the
      // device is faulty.
      UINT status = fx_media_flush(&media);
      if (status != FX_SUCCESS) {
        state = StorageState::PeripheralError;
        break;
      }

      tx_thread_sleep(poll_interval_ticks);
    }

    tx_event_flags_set(&flags, ~storage_ready_flag, TX_AND);

    // If this fails something is seriously wrong.
    fx_media_abort(&media);
  }
}
