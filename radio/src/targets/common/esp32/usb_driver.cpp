/*
 * Copyright (C) EdgeTX
 *
 * Based on code named
 *   opentx - https://github.com/opentx/opentx
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

#include "hal/usb_driver.h"
#include "hal.h"
#include "hal/storage.h"
#include "debug.h"
#include "sdcard.h"
#include "usb_joystick.h"

#if defined(ESP_PLATFORM) && defined(CONFIG_TINYUSB_MSC_ENABLED)
#include "tinyusb.h"
#include "tinyusb_msc.h"
#include "diskio_spi.h"
#include "tusb.h"
#include "esp_rom_sys.h"
// Disable the USB_SERIAL_JTAG hardware chip-reset signal before TinyUSB
// switches the USB_WRAP PHY mux to OTG (which would otherwise cause rst:0x15).
#include "hal/usb_serial_jtag_ll.h"
#include "soc/rtc_cntl_reg.h"

static bool s_tusb_installed = false;
static tinyusb_msc_storage_handle_t s_msc_storage = NULL;
#endif

static usbMode selectedUsbMode = USB_UNSELECTED_MODE;
static bool usbDriverStarted = false;
static bool cdcActive = false;

int getSelectedUsbMode()
{
    return selectedUsbMode;
}

void setSelectedUsbMode(int mode)
{
    selectedUsbMode = usbMode(mode);
}

void usbInit()
{
    usbDriverStarted = false;
}

void usbStart()
{
    usbInit();
    esp_rom_printf("USB: usbStart mode=%d\n", getSelectedUsbMode());

    if (getSelectedUsbMode() == USB_UNSELECTED_MODE) {
        usbDriverStarted = false;
        return;
    }

    switch (getSelectedUsbMode()) {
        case USB_JOYSTICK_MODE:
            if (!setupUSBJoystick()) {
                TRACE("USB joystick setup failed");
            }
            break;

        case USB_MASS_STORAGE_MODE:
#if defined(ESP_PLATFORM) && defined(CONFIG_TINYUSB_MSC_ENABLED)
            if (!s_tusb_installed) {
                // Prevent USB_SERIAL_JTAG from issuing a hardware chip reset
                // (rst:0x15) when the USB_WRAP PHY mux switches to OTG below.
                REG_SET_BIT(RTC_CNTL_USB_CONF_REG, RTC_CNTL_USB_RESET_DISABLE);
                usb_serial_jtag_ll_disable_intr_mask(UINT32_MAX);

                // Correct init order (same as component test app and reference examples):
                // Step 1 — MSC class driver
                tinyusb_msc_driver_config_t msc_cfg = {};
                msc_cfg.user_flags.auto_mount_off = 1;
                esp_err_t err = tinyusb_msc_install_driver(&msc_cfg);
                esp_rom_printf("USB: msc_install_driver=0x%x\n", err);
                if (err != ESP_OK) break;

                // Step 2 — storage (SD card), configured BEFORE USB starts
                sdmmc_card_t* sdcard = sdcard_spi_get_card();
                esp_rom_printf("USB: sdcard=%p\n", sdcard);
                if (sdcard) {
                    tinyusb_msc_storage_config_t cfg = {};
                    cfg.medium.card = sdcard;
                    cfg.mount_point = TINYUSB_MSC_STORAGE_MOUNT_USB;
                    cfg.fat_fs.do_not_format = true;
                    err = tinyusb_msc_new_storage_sdmmc(&cfg, &s_msc_storage);
                    esp_rom_printf("USB: msc_new_storage=0x%x handle=%p\n", err, s_msc_storage);
                }

                // Step 3 — start USB stack last, after storage is configured
                // Defaults from TINYUSB_DEFAULT_CONFIG(): FS port, CPU1, prio 5, 4096B stack.
                tinyusb_config_t tusb_cfg = {};
                tusb_cfg.port = TINYUSB_PORT_FULL_SPEED_0;
                tusb_cfg.phy.skip_setup = false;
                tusb_cfg.phy.self_powered = false;
                tusb_cfg.phy.vbus_monitor_io = -1;
                tusb_cfg.task.size = 4096;
                tusb_cfg.task.priority = 5;
                tusb_cfg.task.xCoreID = 1;   // CPU1 (TINYUSB_DEFAULT_TASK_AFFINITY)
                err = tinyusb_driver_install(&tusb_cfg);
                esp_rom_printf("USB: driver_install=0x%x\n", err);
                if (err == ESP_OK) s_tusb_installed = true;
            } else if (!s_msc_storage) {
                // USB stack already running (from a previous MSC session that was
                // stopped via usbStop). Re-create the storage handle.
                sdmmc_card_t* sdcard = sdcard_spi_get_card();
                if (sdcard) {
                    tinyusb_msc_storage_config_t cfg = {};
                    cfg.medium.card = sdcard;
                    cfg.mount_point = TINYUSB_MSC_STORAGE_MOUNT_USB;
                    cfg.fat_fs.do_not_format = true;
                    esp_err_t err = tinyusb_msc_new_storage_sdmmc(&cfg, &s_msc_storage);
                    esp_rom_printf("USB: re-create storage=0x%x handle=%p\n", err, s_msc_storage);
                    if (err == ESP_OK) {
                        tud_disconnect();
                        vTaskDelay(pdMS_TO_TICKS(250));
                        tud_connect();
                    }
                }
            }
#endif
            break;

#if defined(USB_SERIAL)
        case USB_SERIAL_MODE:
            cdcActive = true;
            break;
#endif

        default:
            break;
    }

    usbDriverStarted = (getSelectedUsbMode() != USB_UNSELECTED_MODE);
}

void usbStop()
{
#if defined(ESP_PLATFORM) && defined(CONFIG_TINYUSB_MSC_ENABLED)
    if (s_msc_storage) {
        tinyusb_msc_delete_storage(s_msc_storage);
        s_msc_storage = NULL;
    }
    if (!sdMounted()) {
        sdInit();
    }
#endif

    cdcActive = false;
    usbDriverStarted = false;
    setSelectedUsbMode(USB_UNSELECTED_MODE);
}

bool usbStarted()
{
    return usbDriverStarted;
}

uint32_t usbSerialFreeSpace()
{
    return 0;
}

void usbJoystickUpdate()
{
}

#if defined(USB_SERIAL)
void usbSerialPutc(void*, uint8_t c)
{
    (void)c;
}

static void* usbSerialInit(void*, const etx_serial_init*)
{
    cdcActive = true;
    return (void*)1;
}

static const etx_serial_driver_t usbSerialDriver = {
    .init = usbSerialInit,
    .deinit = nullptr,
    .sendByte = usbSerialPutc,
    .sendBuffer = nullptr,
    .waitForTxCompleted = nullptr,
    .getByte = nullptr,
    .clearRxBuffer = nullptr,
    .getBaudrate = nullptr,
    .setReceiveCb = nullptr,
    .setBaudrateCb = nullptr,
};

const etx_serial_port_t UsbSerialPort = {
    "USB-VCP",
    &usbSerialDriver,
    nullptr,
    nullptr,
};
#endif
