/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x 
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "edgetx.h"
#include "diskio_spi.h"

#include "driver/gpio.h"
#include "soc/soc_caps.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"


static bool card_present = false;
static bool spi_bus_initialized = false;
static sdmmc_host_t config = SDSPI_HOST_DEFAULT();
static sdspi_dev_handle_t handle;
static sdspi_device_config_t dev_config = SDSPI_DEVICE_CONFIG_DEFAULT();
static sdmmc_card_t sdcard;
static sdmmc_card_t* card = &sdcard;

// Prefer 40 MHz SPI (SDMMC_FREQ_HIGHSPEED). SDSPI_HOST_DEFAULT() is only
// 20 MHz. Fall back to 20 MHz when a card fails high-speed init — some
// cheap cards / long traces cannot sustain 40 MHz.
#ifndef SDCARD_SPI_FREQ_KHZ
#define SDCARD_SPI_FREQ_KHZ SDMMC_FREQ_HIGHSPEED
#endif
#ifndef SDCARD_SPI_FREQ_FALLBACK_KHZ
#define SDCARD_SPI_FREQ_FALLBACK_KHZ SDMMC_FREQ_DEFAULT
#endif

static bool sdcard_spi_try_init(int freq_khz)
{
    config.max_freq_khz = freq_khz;
    config.slot = handle;
    if (sdmmc_card_init(&config, card) == 0) {
        return true;
    }
    return false;
}

static DSTATUS sdcard_spi_initialize(BYTE lun)
{
    if (!card_present) {
#ifdef SD_DEDICATED_SPI
        if (!spi_bus_initialized) {
            spi_bus_config_t bus_config = {
                .mosi_io_num = SDSPI_MOSI,
                .miso_io_num = SDSPI_MISO,
                .sclk_io_num = SDSPI_CLK,
                .quadwp_io_num = -1,
                .quadhd_io_num = -1,
                // Allow multi-sector DMA bursts (default is often too small).
                .max_transfer_sz = 16 * 512,
            };
            config.slot = SD_SPI_HOST;
            if (spi_bus_initialize((spi_host_device_t)config.slot, &bus_config, SPI_DMA_CH_AUTO) != ESP_OK) {
                return STA_NOINIT;
            }
            spi_bus_initialized = true;
        } else {
            // SPI bus already initialized; restore host slot id before reuse.
            config.slot = SD_SPI_HOST;
        }
#endif
        dev_config.host_id = (spi_host_device_t)config.slot;
        dev_config.gpio_cs = SDCARD_CS_GPIO;
        sdspi_host_init();
        if (sdspi_host_init_device(&dev_config, &handle) != ESP_OK) {
            sdspi_host_deinit();
            return STA_NODISK | STA_NOINIT;
        }

        if (sdcard_spi_try_init(SDCARD_SPI_FREQ_KHZ) ||
            (SDCARD_SPI_FREQ_KHZ != SDCARD_SPI_FREQ_FALLBACK_KHZ &&
             sdcard_spi_try_init(SDCARD_SPI_FREQ_FALLBACK_KHZ))) {
            card_present = true;
        } else {
            sdspi_host_remove_device(handle);
            sdspi_host_deinit();
        }
    }
    // Return STA_NODISK|STA_NOINIT when no card so f_mount returns FR_NOT_READY cleanly.
    return card_present ? 0 : (STA_NODISK | STA_NOINIT);
}

static DSTATUS sdcard_spi_status(BYTE lun)
{
    // STA_NOINIT (not STA_NODISK) so storageIsPresent() still returns true and
    // mount attempts are not suppressed when the card is simply absent.
    return card_present ? 0 : STA_NOINIT;
}

static DRESULT sdcard_spi_read(BYTE lun, BYTE * buff, DWORD sector, UINT count)
{
    // TRACE("disk_read %d %p %10d %d", lun, buff, sector, count);

    DRESULT state = RES_OK;
    if (0 != sdmmc_read_sectors(card, buff, sector, count)) {
        state = RES_ERROR;
    }

    return state;
}

static DRESULT sdcard_spi_write(BYTE lun, const BYTE* buff, DWORD sector, UINT count)
{
    // TRACE("disk_write %d %p %10d %d", lun, buff, sector, count);

    DRESULT res = RES_OK;
    if (0 != sdmmc_write_sectors(card, buff, sector, count)) {
        res = RES_ERROR;
    }
    return res;
}

static DRESULT sdcard_spi_ioctl(BYTE lun, BYTE ctrl, void *buff)
{
    assert(card);
    switch(ctrl) {
    case CTRL_SYNC:
        return RES_OK;
    case GET_SECTOR_COUNT:
        *((DWORD*) buff) = card->csd.capacity;
        return RES_OK;
    case GET_SECTOR_SIZE:
        *((WORD*) buff) = card->csd.sector_size;
        return RES_OK;
    case GET_BLOCK_SIZE:
        return RES_ERROR;
    }
    return RES_ERROR;
}

const diskio_driver_t sdcard_spi_driver = {
    .initialize = sdcard_spi_initialize,
    .status = sdcard_spi_status,
    .read = sdcard_spi_read,
    .write = sdcard_spi_write,
    .ioctl = sdcard_spi_ioctl,
};

sdmmc_card_t* sdcard_spi_get_card(void)
{
    return card_present ? card : NULL;
}

