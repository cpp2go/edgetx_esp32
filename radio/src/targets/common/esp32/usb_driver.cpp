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

#if defined(ESP_PLATFORM) && defined(CONFIG_TINYUSB_ENABLED)
#include "tusb.h"
#include "class/cdc/cdc_device.h"
#include "class/msc/msc_device.h"
#include "class/hid/hid_device.h"
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

#if defined(ESP_PLATFORM) && defined(CONFIG_TINYUSB_ENABLED)
extern "C" void tud_mount_cb(void) {}
extern "C" void tud_umount_cb(void) {}
extern "C" void tud_suspend_cb(bool) {}
extern "C" void tud_resume_cb(void) {}

extern "C" void tud_cdc_rx_cb(uint8_t) {}

extern "C" void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
{
    (void) lun;
    memcpy(vendor_id, "EdgeTX  ", 8);
    memcpy(product_id, "MASS STORAGE    ", 16);
    memcpy(product_rev, "1.00", 4);
}

extern "C" bool tud_msc_is_ready_cb(uint8_t lun)
{
    (void) lun;
    return SD_CARD_PRESENT();
}

extern "C" bool tud_msc_is_write_protected_cb(uint8_t lun)
{
    (void) lun;
    return false;
}

extern "C" void tud_msc_capacity_cb(uint8_t lun, uint32_t* block_count, uint16_t* block_size)
{
    (void) lun;
    *block_size = 512;
    *block_count = 0;

    if (SD_CARD_PRESENT()) {
        auto drv = storageGetDefaultDriver();
        if (drv && drv->ioctl(0, GET_SECTOR_COUNT, block_count) == RES_OK) {
            // ok
        } else {
            *block_count = 0;
        }
    }
}

extern "C" int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize)
{
    (void) lun;
    (void) offset;

    if (!SD_CARD_PRESENT()) return -1;

    auto drv = storageGetDefaultDriver();
    if (!drv) return -1;

    uint32_t blocks = bufsize / 512;
    if (drv->read(0, (BYTE*)buffer, lba, (UINT)blocks) != RES_OK) {
        return -1;
    }
    return bufsize;
}

extern "C" int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, const void* buffer, uint32_t bufsize)
{
    (void) lun;
    (void) offset;

    if (!SD_CARD_PRESENT()) return -1;

    auto drv = storageGetDefaultDriver();
    if (!drv) return -1;

    uint32_t blocks = bufsize / 512;
    if (drv->write(0, (const BYTE*)buffer, lba, (UINT)blocks) != RES_OK) {
        return -1;
    }
    return bufsize;
}

extern "C" void tud_msc_scsi_cb(uint8_t lun, const uint8_t* scsi_cmd, void* buffer, uint16_t bufsize)
{
    (void) lun;
    (void) scsi_cmd;
    (void) buffer;
    (void) bufsize;
}
#endif

void usbInit()
{
#if defined(ESP_PLATFORM) && defined(CONFIG_TINYUSB_ENABLED)
    tud_init(0);
#endif
    usbDriverStarted = false;
}

void usbStart()
{
    usbInit();

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
            if (!SD_CARD_PRESENT()) {
                TRACE("USB mass storage selected but no SD card");
            } else {
                storageInit();
            }
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
#if defined(ESP_PLATFORM) && defined(CONFIG_TINYUSB_ENABLED)
    if (tud_ready()) {
        tud_disconnect();
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
#if defined(ESP_PLATFORM) && defined(CONFIG_TINYUSB_ENABLED)
    if (!cdcActive) return 0;
    return tud_cdc_write_available();
#else
    return 0;
#endif
}

void usbJoystickUpdate()
{
    if (!usbStarted() || getSelectedUsbMode() != USB_JOYSTICK_MODE) return;

#if defined(ESP_PLATFORM) && defined(CONFIG_TINYUSB_ENABLED)
    if (tud_hid_ready()) {
        struct usbReport_t report = usbReport();
        tud_hid_report(0, 0, report.ptr, report.size);
    }
#endif
}

#if defined(USB_SERIAL)
void usbSerialPutc(void*, uint8_t c)
{
#if defined(ESP_PLATFORM) && defined(CONFIG_TINYUSB_ENABLED)
    if (!cdcActive || !tud_cdc_connected()) return;
    tud_cdc_write_char((char)c);
    tud_cdc_write_flush();
#else
    (void)c;
#endif
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

