/**************************************************************************/
/*                                                                        */
/*       Partial Copyright (c) Microsoft Corporation. All rights reserved.*/
/*                                                                        */
/*       This software is licensed under the Microsoft Software License   */
/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
/*       and in the root directory of this software.                      */
/*      Partial Copyright (c) STMicroelctronics 2020. All rights reserved */
/**************************************************************************/

/* Include necessary system files.  */
#include "fx_propulse_sd_driver.h"

/*
 * the scratch buffer is required when performing DMA transfers using unaligned addresses
 * When CPU cache is enabled, the scratch buffer should be 32-byte aligned to match a whole cache
 * line otherwise it is 4-byte aligned to match the DMA alignment constraints
 */

#if (FX_PROPULSE_SD_CACHE_MAINTENANCE == 1)
static UCHAR scratch[FX_PROPULSE_SD_SCRATCH_SIZE]
    __attribute__((aligned(32), section(".StorageMemory")));
#else
static UCHAR scratch[FX_PROPULSE_SD_SCRATCH_SIZE]
    __attribute__((aligned(4), section(".StorageMemory")));
#endif

UINT _fx_partition_offset_calculate(void *partition_sector, UINT partition, ULONG *partition_start,
                                    ULONG *partition_size);

static UINT sd_read_data(FX_MEDIA *media_ptr, ULONG sector, UINT num_sectors,
                         UINT use_scratch_buffer);
static UINT sd_write_data(FX_MEDIA *media_ptr, ULONG sector, UINT num_sectors,
                          UINT use_scratch_buffer);

static INT check_sd_status(void) {
  uint32_t start = FX_PROPULSE_SD_CURRENT_TIME();

  /* Clear CMDTRANS before polling the card. The HAL leaves this bit set after
   * a data transfer (it is not part of SDMMC's CMD_CLEAR_MASK), and a stale
   * CMDTRANS poisons the following CMD13/SEND_STATUS poll: the command state
   * machine waits on a data phase that never comes and the response read times
   * out. Clearing it here keeps the readiness poll reliable. */
  __SDMMC_CMDTRANS_DISABLE(hsd1.Instance);

  while (FX_PROPULSE_SD_CURRENT_TIME() - start < FX_PROPULSE_SD_TIMEOUT) {
    if (fx_propulse_sd_get_status() == 0) {
      return 0;
    }
  }

  return 1;
}

/**
 * @brief This function is the entry point to the STM32 SD disk driver.
 * It relies on the STM32 peripheral library from ST.
 * @param media_ptr: FileX's Media Config Block
 * @retval None
 */
VOID fx_propulse_sd_driver(FX_MEDIA *media_ptr) {
  UINT status;
  UINT unaligned_buffer;
  ULONG partition_start;
  ULONG partition_size;

  /* the SD DMA requires a 4-byte aligned buffers */
  unaligned_buffer = (UINT)(media_ptr->fx_media_driver_buffer) & 0x3;

  /* Process the driver request specified in the media control block.  */
  switch (media_ptr->fx_media_driver_request) {
  case FX_DRIVER_INIT: {
    media_ptr->fx_media_driver_status = FX_SUCCESS;

    FX_PROPULSE_SD_PRE_INIT(media_ptr);
    break;
  }

  case FX_DRIVER_UNINIT: {
    media_ptr->fx_media_driver_status = FX_SUCCESS;

    FX_PROPULSE_SD_POST_DEINIT(media_ptr);
    break;
  }

  case FX_DRIVER_READ: {
    media_ptr->fx_media_driver_status = FX_IO_ERROR;

    if (sd_read_data(media_ptr,
                     media_ptr->fx_media_driver_logical_sector + media_ptr->fx_media_hidden_sectors,
                     media_ptr->fx_media_driver_sectors, unaligned_buffer) == FX_SUCCESS) {
      media_ptr->fx_media_driver_status = FX_SUCCESS;
    }

    break;
  }

  case FX_DRIVER_WRITE: {
    media_ptr->fx_media_driver_status = FX_IO_ERROR;

    if (sd_write_data(media_ptr,
                      media_ptr->fx_media_driver_logical_sector +
                          media_ptr->fx_media_hidden_sectors,
                      media_ptr->fx_media_driver_sectors, unaligned_buffer) == FX_SUCCESS) {
      media_ptr->fx_media_driver_status = FX_SUCCESS;
    }

    break;
  }

  case FX_DRIVER_FLUSH: {
    /* Return driver success.  */
    media_ptr->fx_media_driver_status = FX_SUCCESS;
    break;
  }

  case FX_DRIVER_ABORT: {
    /* Return driver success.  */
    media_ptr->fx_media_driver_status = FX_SUCCESS;

    FX_PROPULSE_SD_POST_DEINIT(media_ptr);
    break;
  }

  case FX_DRIVER_BOOT_READ: {
    /* the boot sector is the sector zero */

    /* before performing any operation, check the status of the SD IP */
    if (check_sd_status() != 0) {
      media_ptr->fx_media_driver_status = FX_IO_ERROR;
      return;
    }

    status = sd_read_data(media_ptr, 0, media_ptr->fx_media_driver_sectors, unaligned_buffer);

    if (status != FX_SUCCESS) {
      media_ptr->fx_media_driver_status = status;
      break;
    }

    /* Check if the sector 0 is the actual boot sector, otherwise calculate the offset into it.
    Please note that this should belong to higher level of MW to do this check and it is here
    as a temporary work solution */

    partition_start = 0;

    status = _fx_partition_offset_calculate(media_ptr->fx_media_driver_buffer, 0, &partition_start,
                                            &partition_size);

    /* Check partition read error.  */
    if (status) {
      /* Unsuccessful driver request.  */
      media_ptr->fx_media_driver_status = FX_IO_ERROR;
      break;
    }

    /* Now determine if there is a partition...   */
    if (partition_start) {

      if (check_sd_status() != 0) {
        media_ptr->fx_media_driver_status = FX_IO_ERROR;
        break;
      }

      /* Yes, now lets read the actual boot record.  */
      status = sd_read_data(media_ptr, partition_start, media_ptr->fx_media_driver_sectors,
                            unaligned_buffer);

      if (status != FX_SUCCESS) {
        media_ptr->fx_media_driver_status = status;
        break;
      }
    }

    /* Successful driver request.  */
    media_ptr->fx_media_driver_status = FX_SUCCESS;
    break;
  }

  case FX_DRIVER_BOOT_WRITE: {
    /* before performing any operation, check the status of the SD IP */
    if (check_sd_status() != 0) {
      media_ptr->fx_media_driver_status = FX_IO_ERROR;
      return;
    }

    status = sd_write_data(media_ptr, 0, media_ptr->fx_media_driver_sectors, unaligned_buffer);

    media_ptr->fx_media_driver_status = status;

    break;
  }

  default: {
    media_ptr->fx_media_driver_status = FX_IO_ERROR;
    break;
  }
  }
}

/**
 * @brief Read data from uSD into destination buffer
 * @param FX_MEDIA *media_ptr a pointer the main FileX structure
 * @param ULONG start_sector first sector to start reading from
 * @param UINT num_sectors number of sectors to be read
 * @param UINT use_scratch_buffer to enable scratch buffer usage or not.
 * @retval FX_SUCCESS on success FX_BUFFER_ERROR / FX_ACCESS_ERROR / FX_IO_ERROR otherwise
 */

static UINT sd_read_data(FX_MEDIA *media_ptr, ULONG start_sector, UINT num_sectors,
                         UINT use_scratch_buffer) {
  UINT status;
  UCHAR *read_addr;

  /* perform the Pre read operations */

  if (use_scratch_buffer) {
    UINT remaining = num_sectors;
    read_addr      = media_ptr->fx_media_driver_buffer;

    /* Bounce through the aligned scratch buffer, but stage as many sectors as it
     * holds per iteration so the transfer stays multi-block instead of degrading
     * to one DMA per sector. */
    while (remaining > 0) {
      UINT batch =
          remaining < FX_PROPULSE_SD_SCRATCH_SECTORS ? remaining : FX_PROPULSE_SD_SCRATCH_SECTORS;
      UINT batch_bytes = batch * FX_PROPULSE_SD_SECTOR_SIZE;

      /* Wait for the card to be ready. Strictly this only matters when the
       * previous operation was a write (the card may still be programming),
       * but polling before a read is harmless. */
      if (check_sd_status() != 0)
        return FX_IO_ERROR;

      /* Read the whole batch into the scratch buffer */
      status = fx_propulse_sd_read_blocks((UINT *)scratch, (UINT)start_sector, batch);

      if (status != 0) {
        /* read error occurred, call the error handler code then return immediately */
        return FX_IO_ERROR;
      }

      /* wait for read transfer notification */
      FX_PROPULSE_SD_READ_CPLT_NOTIFY();

#if (FX_PROPULSE_SD_CACHE_MAINTENANCE == 1)
      /* Discard stale cache lines so the CPU copy below sees the DMA'd data */
      invalidate_cache_by_addr((uint32_t *)scratch, batch_bytes);
#endif

      _fx_utility_memory_copy(scratch, read_addr, batch_bytes);
      read_addr += batch_bytes;

      start_sector += batch;
      remaining -= batch;
    }

    status = FX_SUCCESS;
  } else {
    /* Wait for the card to be ready. Strictly this only matters when the
     * previous operation was a write (the card may still be programming),
     * but polling before a read is harmless. */
    if (check_sd_status() != 0)
      return FX_IO_ERROR;

    status = fx_propulse_sd_read_blocks((UINT *)media_ptr->fx_media_driver_buffer,
                                        (UINT)start_sector, num_sectors);

    if (status != 0) {
      /* read error occurred, call the error handler code then return immediately */

      return FX_IO_ERROR;
    }

    /* wait for read transfer notification */
    FX_PROPULSE_SD_READ_CPLT_NOTIFY();

#if (FX_PROPULSE_SD_CACHE_MAINTENANCE == 1)
    invalidate_cache_by_addr((uint32_t *)media_ptr->fx_media_driver_buffer,
                             num_sectors * FX_PROPULSE_SD_SECTOR_SIZE);
#endif

    status = FX_SUCCESS;
  }

  /* Operation finished, call the post read macro if defined */

  return status;
}

/**
 * @brief write data buffer into the uSD
 * @param FX_MEDIA *media_ptr a pointer the main FileX structure
 * @param ULONG start_sector first sector to start writing from
 * @param UINT num_sectors number of sectors to be written
 * @param UINT use_scratch_buffer to enable scratch buffer usage or not.
 * @retval FX_SUCCESS on success FX_BUFFER_ERROR / FX_ACCESS_ERROR / FX_IO_ERROR otherwise
 */

static UINT sd_write_data(FX_MEDIA *media_ptr, ULONG start_sector, UINT num_sectors,
                          UINT use_scratch_buffer) {
  UINT status;
  UCHAR *write_addr;

  /* call Pre write operation macro */

  if (use_scratch_buffer) {
    UINT remaining = num_sectors;
    write_addr     = media_ptr->fx_media_driver_buffer;

    /* Bounce through the aligned scratch buffer, but stage as many sectors as it
     * holds per iteration so the transfer stays multi-block instead of degrading
     * to one DMA per sector. */
    while (remaining > 0) {
      UINT batch =
          remaining < FX_PROPULSE_SD_SCRATCH_SECTORS ? remaining : FX_PROPULSE_SD_SCRATCH_SECTORS;
      UINT batch_bytes = batch * FX_PROPULSE_SD_SECTOR_SIZE;

      /* Stage the batch into the aligned scratch buffer */
      _fx_utility_memory_copy(write_addr, scratch, batch_bytes);
      write_addr += batch_bytes;

#if (FX_PROPULSE_SD_CACHE_MAINTENANCE == 1)
      /* Clean the DCache to make the SD DMA see the actual content of the scratch buffer */
      clean_cache_by_addr((uint32_t *)scratch, batch_bytes);
#endif

      /* Wait for the card to leave its programming/busy state before issuing
       * the next write. The completion notification only fires on DATAEND
       * (data shifted out); the card then holds DAT0 busy while it commits to
       * flash, and a CMD25 issued into that window is rejected. This must be
       * done before every batch, not just once before the loop. The staging
       * copy above deliberately runs first so it overlaps the busy time. */
      if (check_sd_status() != 0)
        return FX_IO_ERROR;

      status = fx_propulse_sd_write_blocks((UINT *)scratch, (UINT)start_sector, batch);

      if (status != 0) {
        return FX_IO_ERROR;
      }

      /* Wait for the write (data-transfer) completion notification */
      FX_PROPULSE_SD_WRITE_CPLT_NOTIFY();

      start_sector += batch;
      remaining -= batch;
    }

    status = FX_SUCCESS;
  } else {
#if (FX_PROPULSE_SD_CACHE_MAINTENANCE == 1)
    clean_cache_by_addr((uint32_t *)media_ptr->fx_media_driver_buffer,
                        num_sectors * FX_PROPULSE_SD_SECTOR_SIZE);
#endif
    /* Wait for the card to leave its programming/busy state before writing;
     * a CMD25 issued while the card is still busy on DAT0 is rejected. */
    if (check_sd_status() != 0) {
      return FX_IO_ERROR;
    }

    status = fx_propulse_sd_write_blocks((UINT *)media_ptr->fx_media_driver_buffer,
                                         (UINT)start_sector, num_sectors);

    if (status != 0) {
      return FX_IO_ERROR;
    }

    /* when defined, wait for the write notification */
    FX_PROPULSE_SD_WRITE_CPLT_NOTIFY();

    status = FX_SUCCESS;
  }

  /* perform post write operations */

  return status;
}
