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
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"

// SD card on the OSPTEK P4C5 dev board TF slot is wired to the SDMMC
// controller (slot 0, 4-bit) on the module's SD1 pads:
//   CLK = GPIO43, CMD = GPIO44, D0 = GPIO39, D1 = GPIO40, D2 = GPIO41,
//   D3 = GPIO42 (see SDMMC_* in the target board.h). Slot 0 uses the fixed
//   IO-MUX pin mapping, so no GPIOs need to be passed to the host driver.

static bool card_present = false;
static bool sdmmc_host_initialized = false;
static sdmmc_host_t config = SDMMC_HOST_DEFAULT();
static sdmmc_card_t sdcard;
static sdmmc_card_t* card = &sdcard;

static DSTATUS sdcard_spi_initialize(BYTE lun)
{
    if (!card_present) {
        if (!sdmmc_host_initialized) {
            config.slot = SDMMC_HOST_SLOT_0;
            config.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

            sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
            slot_config.width = 4;
            slot_config.flags = 0;  // TF slot has its own pull-ups

            if (sdmmc_host_init() == ESP_OK &&
                sdmmc_host_init_slot(SDMMC_HOST_SLOT_0, &slot_config) == ESP_OK) {
                sdmmc_host_initialized = true;
            } else {
                sdmmc_host_deinit();
            }
        }

        if (sdmmc_host_initialized && 0 == sdmmc_card_init(&config, card)) {
            card_present = true;
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

