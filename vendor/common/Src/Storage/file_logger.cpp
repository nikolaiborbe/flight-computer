#include "file_logger.hpp"

// poly = 0x1021, init = 0xFFFF, no input/output reflection, no xorout.
static uint16_t crc16_ccitt_false_update(const uint8_t *data, size_t length, uint16_t crc) {
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000)
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      else
        crc = static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

static uint16_t log_entry_checksum(const LogEntryHeader &header, const uint8_t *data) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&header);
  constexpr size_t header_offset =
      offsetof(LogEntryHeader, checksum) + sizeof(LogEntryHeader::checksum);

  uint16_t crc = 0xFFFF;
  crc =
      crc16_ccitt_false_update(bytes + header_offset, sizeof(LogEntryHeader) - header_offset, crc);
  crc = crc16_ccitt_false_update(data, header.size, crc);
  return crc;
}

FileLogger::FileLogger(StorageManager &storage_manager, const char *logger_name,
                       const char *log_file_name, GPIO_TypeDef *const led_port,
                       const uint16_t led_pin)
    : storage_manager(storage_manager), led_port(led_port), led_pin(led_pin) {
  strncpy(this->logger_name, logger_name, sizeof(this->logger_name) - 1);
  this->logger_name[sizeof(this->logger_name) - 1] = '\0';
  strncpy(this->log_file_name, log_file_name, sizeof(this->log_file_name) - 1);
  this->log_file_name[sizeof(this->log_file_name) - 1] = '\0';
}

UINT FileLogger::init(void) {
  UINT status;

  // tx_queue_create takes the message size in 32-bit words, not bytes.
  static_assert(sizeof(LogEntry) % sizeof(ULONG) == 0);

  status = tx_queue_create(&logger_queue, logger_name, sizeof(LogEntry) / sizeof(ULONG),
                           logger_queue_buffer, sizeof(logger_queue_buffer));
  if (status != TX_SUCCESS)
    return status;

  status = tx_byte_pool_create(&logger_pool, logger_name, logger_pool_buffer,
                               sizeof(logger_pool_buffer));
  if (status != TX_SUCCESS)
    return status;

  status = tx_event_flags_create(&logger_flags, logger_name);
  if (status != TX_SUCCESS)
    return status;

  return status;
}

UINT FileLogger::enqueue(const uint16_t id, const uint16_t data_size, const uint8_t *const data) {
  LogEntry entry;
  UINT status;
  ULONG actual_flags;

  if (!data || !data_size) {
    enqueue_rejected_accum++;
    led_off();
    return TX_PTR_ERROR;
  }

  // Only accept entries once the worker thread has an open file to write to.
  if (tx_event_flags_get(&logger_flags, logger_ready_flag, TX_AND, &actual_flags, TX_NO_WAIT) !=
      TX_SUCCESS) {
    enqueue_rejected_accum++;
    led_off();
    return TX_NOT_AVAILABLE;
  }

  status = tx_byte_allocate(&logger_pool, (VOID **)&entry.data, data_size, TX_NO_WAIT);
  if (status != TX_SUCCESS) {
    enqueue_rejected_accum++;
    led_off();
    return status;
  }

  memcpy(entry.data, data, data_size);

  entry.header = {
      .magic        = logger_magic_number,
      .checksum     = 0,
      .id           = id,
      .size         = data_size,
      .reserved0    = 0,
      .timestamp_us = timebase::micros(),
  };

  // Calculate checksum after all fields have been populated.
  entry.header.checksum = log_entry_checksum(entry.header, entry.data);

  status = tx_queue_send(&logger_queue, &entry, TX_NO_WAIT);
  if (status != TX_SUCCESS) {
    // Byte release should not fail after a succesfull allocate.
    // If it does something has gone seriously wrong.
    tx_byte_release(entry.data);
    enqueue_rejected_accum++;
    led_off();
    return status;
  }

  enqueue_accepted_accum++;
  led_on();
  return status;
}

FileLoggerTelemetry FileLogger::get_telemetry(void) {
  FileLoggerTelemetry t{};
  t.write_bps        = write_bps;
  t.write_us_max     = write_us_max;
  t.enqueue_accepted = enqueue_accepted;
  t.enqueue_rejected = enqueue_rejected;
  t.state            = state;
  return t;
}

void FileLogger::reset_throughput(void) {
  speed_last_us          = timebase::micros();
  write_bytes_accum      = 0;
  write_us_max_accum     = 0;
  enqueue_accepted_accum = 0;
  enqueue_rejected_accum = 0;
  write_bps              = 0;
  write_us_max           = 0;
  enqueue_accepted       = 0;
  enqueue_rejected       = 0;
}

void FileLogger::update_throughput(uint32_t bytes, uint32_t write_us) {
  // Fold this write into the in-progress window.
  write_bytes_accum += bytes;
  if (write_us > write_us_max_accum)
    write_us_max_accum = write_us;

  // Only publish a new sample once the sample interval has elapsed.
  const uint64_t now_us     = timebase::micros();
  const uint64_t elapsed_us = now_us - speed_last_us;
  if (elapsed_us < throughput_sample_interval_us)
    return;

  // A 64-bit intermediate keeps the byte-to-rate scaling from overflowing.
  write_bps        = (uint32_t)((uint64_t)write_bytes_accum * 1000000 / elapsed_us);
  write_us_max     = write_us_max_accum;
  enqueue_accepted = enqueue_accepted_accum;
  enqueue_rejected = enqueue_rejected_accum;

  write_bytes_accum      = 0;
  write_us_max_accum     = 0;
  enqueue_accepted_accum = 0;
  enqueue_rejected_accum = 0;
  speed_last_us          = now_us;
}

// Dequeue one entry and serialise it (header followed by payload) into `dest`.
// On success `*entry_size` holds the number of bytes written.
UINT FileLogger::dequeue(uint8_t *const dest, uint16_t dest_size, uint16_t *const entry_size) {
  LogEntry entry;
  UINT status;

  if (!dest || !entry_size)
    return TX_PTR_ERROR;

  status = tx_queue_receive(&logger_queue, &entry, TX_WAIT_FOREVER);
  if (status != TX_SUCCESS)
    return status;

  const uint16_t total = sizeof(LogEntryHeader) + entry.header.size;
  if (dest_size < total) {
    tx_byte_release(entry.data);
    return TX_SIZE_ERROR;
  }

  memcpy(dest, &entry.header, sizeof(LogEntryHeader));
  memcpy(dest + sizeof(LogEntryHeader), entry.data, entry.header.size);

  // This should honestly not fail.
  tx_byte_release(entry.data);

  *entry_size = total;
  return status;
}

void FileLogger::led_on(void) {
  if (!led_port)
    return;

  HAL_GPIO_WritePin(led_port, led_pin, GPIO_PIN_SET);
}

void FileLogger::led_off(void) {
  if (!led_port)
    return;

  HAL_GPIO_WritePin(led_port, led_pin, GPIO_PIN_RESET);
}

void FileLogger::run(void) {
  while (true) {
    UINT status;

    FX_MEDIA *media;

    status = storage_manager.get_media(&media);
    if (status != TX_SUCCESS) {
      state = FileLoggerState::WaitingForStorage;
      tx_thread_sleep(poll_interval_ticks);
      continue;
    }

    // Create a file to log telemetry to.
    // If it already exists, then all is good.
    status = fx_file_create(media, log_file_name);
    if (status != FX_SUCCESS && status != FX_ALREADY_CREATED) {
      state = FileLoggerState::CreateError;
      tx_thread_sleep(poll_interval_ticks);
      continue;
    }

    // Open the file to log data to.
    status = fx_file_open(media, &telemetry_file, log_file_name, FX_OPEN_FOR_WRITE);
    if (status != FX_SUCCESS) {
      state = FileLoggerState::OpenError;
      tx_thread_sleep(poll_interval_ticks);
      continue;
    }

    // Go to end of file to append to it.
    status = fx_file_relative_seek(&telemetry_file, 0, FX_SEEK_END);
    if (status != FX_SUCCESS) {
      state = FileLoggerState::SeekError;
      fx_file_close(&telemetry_file);
      tx_thread_sleep(poll_interval_ticks);
      continue;
    }

    // Baseline the throughput window for this logging session.
    reset_throughput();

    // The file is open and positioned; start accepting entries.
    state = FileLoggerState::Running;
    tx_event_flags_set(&logger_flags, logger_ready_flag, TX_OR);

    // Keep track of current entry in the telemetry buffer.
    size_t telemetry_buffer_tail = 0;

    while (true) {
      uint16_t entry_size;
      size_t telemetry_buffer_remainig_size = telemetry_file_buffer_size - telemetry_buffer_tail;

      if (dequeue(&telemetry_file_buffer[telemetry_buffer_tail],
                  telemetry_buffer_remainig_size > 0xFFFF ? 0xFFFF : telemetry_buffer_remainig_size,
                  &entry_size) != TX_SUCCESS)
        break;

      telemetry_buffer_tail += entry_size;

      // Don't do anything while there is still space left in the buffer.
      if (telemetry_file_buffer_size - telemetry_buffer_tail >=
          sizeof(LogEntryHeader) + telemetry_file_buffer_ullage_size)
        continue;

      // Write to file once buffer is full, timing the (blocking) call. A large
      // write_us_max vs. a modest write_bps points at SD housekeeping stalls; a
      // consistently low write_bps points at a copy/throughput ceiling instead.
      const uint64_t write_t0 = timebase::micros();
      status = fx_file_write(&telemetry_file, telemetry_file_buffer, telemetry_buffer_tail);
      const uint32_t write_dt = static_cast<uint32_t>(timebase::micros() - write_t0);

      if (status != FX_SUCCESS) {
        state = FileLoggerState::WriteError;
        break;
      }

      update_throughput(static_cast<uint32_t>(telemetry_buffer_tail), write_dt);

      // Begin from the begining of the telemetry buffer again.
      telemetry_buffer_tail = 0;
    }

    // No longer able to accept entries once the write loop has dropped out.
    tx_event_flags_set(&logger_flags, ~logger_ready_flag, TX_AND);

    // If this fail it doesn't matter.
    fx_file_close(&telemetry_file);
  }
}
