/**************************************************************************/
/*                                                                        */
/*       Copyright (c) Microsoft Corporation. All rights reserved.        */
/*                                                                        */
/*       This software is licensed under the Microsoft Software License   */
/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
/*       and in the root directory of this software.                      */
/*                                                                        */
/**************************************************************************/

// This file is copied over and modified from the generated CubeMX middleware for LevelX.

#include "nand_flash_fx.h"

ULONG __attribute__((section(".FxMemory"))) nand_flash_fx_driver_buffer[(7 * NAND_FLASH_TOTAL_BLOCKS + 4 + 2 * (NAND_FLASH_BYTES_PER_PHYSICAL_PAGE + NAND_FLASH_SPARE_BYTES_PER_PAGE)) / sizeof(ULONG)];

VOID  nand_flash_fx_driver(FX_MEDIA *media_ptr, LX_NAND_FLASH *lx_nand_flash, OSPI_HandleTypeDef *hospi, char *name)
{
    ULONG i;
    UINT status;
    UCHAR *source_buffer;
    UCHAR *destination_buffer;
    ULONG logical_sector;

    switch(media_ptr->fx_media_driver_request)
    {

        case FX_DRIVER_INIT:
            {
                /* Format flash instance*/
                status = lx_nand_flash_format(lx_nand_flash, name, nand_flash_lx_initialize, nand_flash_fx_driver_buffer, sizeof(nand_flash_fx_driver_buffer));
                if (status != LX_SUCCESS)
                {
                    media_ptr->fx_media_driver_status = FX_IO_ERROR;

                    return;
                }
                /* Open flash instance*/
                status = lx_nand_flash_open(lx_nand_flash, name, nand_flash_lx_initialize, nand_flash_fx_driver_buffer, sizeof(nand_flash_fx_driver_buffer));

                /* LevelX driver correctly initialized */
                if (status == LX_SUCCESS)
                {
                    media_ptr->fx_media_driver_status = FX_SUCCESS;

                    media_ptr->fx_media_driver_free_sector_update = FX_TRUE;
                }
                else
                {
                    media_ptr->fx_media_driver_status = FX_IO_ERROR;
                }

                break;
            }

        case FX_DRIVER_UNINIT:
            {
                /* Successful driver */
                status = lx_nand_flash_close(lx_nand_flash);

                if (status == LX_SUCCESS)
                {
                    media_ptr->fx_media_driver_status =  FX_SUCCESS;
                }
                else
                {
                    media_ptr->fx_media_driver_status =  FX_IO_ERROR;
                }

                break;
            }

        case FX_DRIVER_READ:
            {
                /* Setup the destination buffer and logical sector.  */
                logical_sector = media_ptr->fx_media_driver_logical_sector;
                destination_buffer =(UCHAR *)media_ptr->fx_media_driver_buffer;

                /* Loop to read sectors from flash.  */
                for (i = 0; i < media_ptr->fx_media_driver_sectors; i++)
                {

                    /* Read a sector from NAND flash.  */
                    status =  lx_nand_flash_sector_read(lx_nand_flash, logical_sector, destination_buffer);

                    /* Determine if the read was successful.  */
                    if (status != LX_SUCCESS)
                    {

                        /* Return an I/O error to FileX.  */
                        media_ptr->fx_media_driver_status = FX_IO_ERROR;

                        return;
                    }

                    /* Move to the next entries.  */
                    logical_sector++;
                    destination_buffer = destination_buffer + media_ptr->fx_media_bytes_per_sector;
                }

                /* Successful driver request.  */
                media_ptr->fx_media_driver_status =  FX_SUCCESS;

                break;
            }

        case FX_DRIVER_BOOT_READ:
            {

                /* Read the boot record and return to the caller.  */

                /* Setup the destination buffer.  */
                destination_buffer =  (UCHAR *) media_ptr -> fx_media_driver_buffer;

                /* Read boot sector from NAND flash.  */
                status =  lx_nand_flash_sector_read(lx_nand_flash, 0, destination_buffer);

                /* Determine if the boot read was successful.  */
                if (status != LX_SUCCESS)
                {
                    /* Return an I/O error to FileX.  */
                    media_ptr -> fx_media_driver_status =  FX_IO_ERROR;

                    return;
                }

                /* Successful driver request.  */
                media_ptr -> fx_media_driver_status =  FX_SUCCESS;
                break;
            }

        case FX_DRIVER_WRITE:
            {
                /* Setup the source buffer and logical sector.  */
                logical_sector = media_ptr->fx_media_driver_logical_sector;
                source_buffer = (UCHAR *) media_ptr->fx_media_driver_buffer;

                /* Loop to write sectors to flash.  */
                for (i = 0; i < media_ptr->fx_media_driver_sectors; i++)
                {
                    /* Write a sector to NAND flash.  */
                    status =  lx_nand_flash_sector_write(lx_nand_flash, logical_sector, source_buffer);

                    /* Determine if the write was successful.  */
                    if (status != LX_SUCCESS)
                    {
                        /* Return an I/O error to FileX.  */
                        media_ptr->fx_media_driver_status =  FX_IO_ERROR;
                        return;
                    }

                    /* Move to the next entries.  */
                    logical_sector++;
                    source_buffer =  source_buffer + media_ptr->fx_media_bytes_per_sector;
                }

                /* Successful driver request.  */
                media_ptr->fx_media_driver_status =  FX_SUCCESS;
                break;
            }

        case FX_DRIVER_BOOT_WRITE:
            {

                /* Write the boot record and return to the caller.  */

                /* Setup the source buffer.  */
                source_buffer =       (UCHAR *) media_ptr -> fx_media_driver_buffer;

                /* Write boot sector to NAND flash.  */
                status =  lx_nand_flash_sector_write(lx_nand_flash, 0, source_buffer);

                /* Determine if the boot write was successful.  */
                if (status != LX_SUCCESS)
                {

                    /* Return an I/O error to FileX.  */
                    media_ptr -> fx_media_driver_status =  FX_IO_ERROR;

                    return;
                }

                /* Successful driver request.  */
                media_ptr -> fx_media_driver_status =  FX_SUCCESS;
                break ;
            }
        case FX_DRIVER_RELEASE_SECTORS:
            {
                /* Setup the logical sector.  */
                logical_sector =  media_ptr->fx_media_driver_logical_sector;

                /* Release sectors.  */
                for (i = 0; i < media_ptr->fx_media_driver_sectors; i++)
                {
                    /* Release NAND flash sector.  */
                    status = lx_nand_flash_sector_release(lx_nand_flash, logical_sector);

                    /* Determine if the sector release was successful.  */
                    if (status != LX_SUCCESS)
                    {
                        /* Return an I/O error to FileX.  */
                        media_ptr->fx_media_driver_status = FX_IO_ERROR;
                        return;
                    }

                    /* Move to the next entries.  */
                    logical_sector++;
                }

                /* Successful driver request.  */
                media_ptr->fx_media_driver_status =  FX_SUCCESS;
                break;
            }

        case FX_DRIVER_FLUSH:
            {
                /* Return driver success.  */
                media_ptr->fx_media_driver_status =  FX_SUCCESS;
                break;
            }

        case FX_DRIVER_ABORT:
            {
                /* Return driver success.  */
                media_ptr->fx_media_driver_status =  FX_SUCCESS;
                break;
            }

        default:
            {
                /* Invalid driver request.  */
                media_ptr->fx_media_driver_status =  FX_IO_ERROR;
                break;
            }
    }
}
