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

#if defined(ESP_PLATFORM)
#include "tinyusb.h"
#include "tusb.h"
#include "esp_rom_sys.h"
// Disable the USB_SERIAL_JTAG hardware chip-reset signal before TinyUSB
// switches the USB_WRAP PHY mux to OTG (which would otherwise cause rst:0x15).
#include "hal/usb_serial_jtag_ll.h"
#include "soc/rtc_cntl_reg.h"

static bool s_tusb_installed = false;
static uint8_t s_hid_cfg_desc[64] = {};
#if defined(CONFIG_TINYUSB_MSC_ENABLED)
#include "tinyusb_msc.h"
#include "diskio_spi.h"
static tinyusb_msc_storage_handle_t s_msc_storage = NULL;
#endif
#if defined(CONFIG_TINYUSB_CDC_ENABLED)
#include "tinyusb_cdc_acm.h"
static bool s_cdc_acm_inited = false;
#endif
#endif

static usbMode selectedUsbMode = USB_UNSELECTED_MODE;
static bool usbDriverStarted = false;
static bool cdcActive = false;

#if defined(ESP_PLATFORM)
static void teardownTinyUsbStack()
{
#if defined(CONFIG_TINYUSB_CDC_ENABLED)
    if (s_cdc_acm_inited) {
        tinyusb_cdcacm_deinit(TINYUSB_CDC_ACM_0);
        s_cdc_acm_inited = false;
    }
#endif

    if (s_tusb_installed) {
        tud_disconnect();
        vTaskDelay(pdMS_TO_TICKS(50));
        tinyusb_driver_uninstall();
        s_tusb_installed = false;
    }

#if defined(CONFIG_TINYUSB_MSC_ENABLED)
    if (s_msc_storage) {
        tinyusb_msc_delete_storage(s_msc_storage);
        s_msc_storage = NULL;
    }
#endif
}

extern "C" uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    if (!usbJoystickActive()) {
        setupUSBJoystick();
    }
    usbReport_t report = usbReportDesc();
    return report.ptr;
}

extern "C" uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;
    return 0;
}

extern "C" void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)bufsize;
}
#endif

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
    usbPlugged();
}

void usbStart()
{
    usbInit();
    esp_rom_printf("USB: usbStart mode=%d\n", getSelectedUsbMode());
    bool startOk = false;

    if (getSelectedUsbMode() == USB_UNSELECTED_MODE) {
        usbDriverStarted = false;
        return;
    }

#if defined(ESP_PLATFORM)
    teardownTinyUsbStack();
#endif

    switch (getSelectedUsbMode()) {
        case USB_JOYSTICK_MODE: {
#if defined(ESP_PLATFORM)
            bool joystickReady = setupUSBJoystick();
            if (!joystickReady) {
                TRACE("USB joystick setup failed");
            }

            if (!s_tusb_installed) {
                REG_SET_BIT(RTC_CNTL_USB_CONF_REG, RTC_CNTL_USB_RESET_DISABLE);
                usb_serial_jtag_ll_disable_intr_mask(UINT32_MAX);

                if (!setupUSBJoystick()) {
                    TRACE("USB joystick setup failed");
                }

                uint8_t report_desc_len = usbJoystickReportDescSize();
                if (report_desc_len > 0) {
                    memset(s_hid_cfg_desc, 0, sizeof(s_hid_cfg_desc));
                    uint16_t total_len = TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN;
                    s_hid_cfg_desc[0] = 9;
                    s_hid_cfg_desc[1] = TUSB_DESC_CONFIGURATION;
                    s_hid_cfg_desc[2] = (uint8_t)(total_len & 0xff);
                    s_hid_cfg_desc[3] = (uint8_t)((total_len >> 8) & 0xff);
                    s_hid_cfg_desc[4] = 1;
                    s_hid_cfg_desc[5] = 1;
                    s_hid_cfg_desc[6] = 0;
                    s_hid_cfg_desc[7] = 0x80;
                    s_hid_cfg_desc[8] = 100;

                    s_hid_cfg_desc[9] = 9;
                    s_hid_cfg_desc[10] = TUSB_DESC_INTERFACE;
                    s_hid_cfg_desc[11] = 0;
                    s_hid_cfg_desc[12] = 0;
                    s_hid_cfg_desc[13] = 1;
                    s_hid_cfg_desc[14] = TUSB_CLASS_HID;
                    s_hid_cfg_desc[15] = 0;
                    s_hid_cfg_desc[16] = 0;
                    s_hid_cfg_desc[17] = 0;

                    s_hid_cfg_desc[18] = 9;
                    s_hid_cfg_desc[19] = HID_DESC_TYPE_HID;
                    s_hid_cfg_desc[20] = 0x11;
                    s_hid_cfg_desc[21] = 0x01;
                    s_hid_cfg_desc[22] = 0;
                    s_hid_cfg_desc[23] = 1;
                    s_hid_cfg_desc[24] = HID_DESC_TYPE_REPORT;
                    s_hid_cfg_desc[25] = (uint8_t)(report_desc_len & 0xff);
                    s_hid_cfg_desc[26] = (uint8_t)((report_desc_len >> 8) & 0xff);

                    s_hid_cfg_desc[27] = 7;
                    s_hid_cfg_desc[28] = TUSB_DESC_ENDPOINT;
                    s_hid_cfg_desc[29] = 0x81;
                    s_hid_cfg_desc[30] = TUSB_XFER_INTERRUPT;
                    s_hid_cfg_desc[31] = 16;
                    s_hid_cfg_desc[32] = 0;
                    s_hid_cfg_desc[33] = 10;
                }

                tinyusb_config_t tusb_cfg = {};
                tusb_cfg.port = TINYUSB_PORT_FULL_SPEED_0;
                tusb_cfg.phy.skip_setup = false;
                tusb_cfg.phy.self_powered = false;
                tusb_cfg.phy.vbus_monitor_io = -1;
                tusb_cfg.task.size = 4096;
                tusb_cfg.task.priority = 5;
                tusb_cfg.task.xCoreID = 1;
                tusb_cfg.descriptor.full_speed_config = s_hid_cfg_desc;
                tusb_cfg.descriptor.high_speed_config = s_hid_cfg_desc;

                esp_err_t err = tinyusb_driver_install(&tusb_cfg);
                esp_rom_printf("USB: joystick driver_install=0x%x\n", err);
                if (err == ESP_OK) {
                    s_tusb_installed = true;
                    tud_connect();
                }
            }

            startOk = (joystickReady && s_tusb_installed);
#else
            startOk = setupUSBJoystick();
            if (!startOk) {
                TRACE("USB joystick setup failed");
            }
#endif
            break;
        }

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
                if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) break;

                // Step 2 — storage (SD card), configured BEFORE USB starts
                sdmmc_card_t* sdcard = sdcard_spi_get_card();
                if (!sdcard) {
                    sdInit();
                    sdcard = sdcard_spi_get_card();
                }
                esp_rom_printf("USB: sdcard=%p\n", sdcard);
                if (!sdcard) break;

                tinyusb_msc_storage_config_t cfg = {};
                cfg.medium.card = sdcard;
                cfg.mount_point = TINYUSB_MSC_STORAGE_MOUNT_USB;
                cfg.fat_fs.do_not_format = true;
                err = tinyusb_msc_new_storage_sdmmc(&cfg, &s_msc_storage);
                esp_rom_printf("USB: msc_new_storage=0x%x handle=%p\n", err, s_msc_storage);
                if (err != ESP_OK || !s_msc_storage) break;

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
                if (!sdcard) {
                    sdInit();
                    sdcard = sdcard_spi_get_card();
                }
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

            startOk = (s_tusb_installed && s_msc_storage != NULL);
#endif
            break;

#if defined(USB_SERIAL)
        case USB_SERIAL_MODE:
#if defined(ESP_PLATFORM)
            REG_SET_BIT(RTC_CNTL_USB_CONF_REG, RTC_CNTL_USB_RESET_DISABLE);
            usb_serial_jtag_ll_disable_intr_mask(UINT32_MAX);

            if (!s_tusb_installed) {
                tinyusb_config_t tusb_cfg = {};
                tusb_cfg.port = TINYUSB_PORT_FULL_SPEED_0;
                tusb_cfg.phy.skip_setup = false;
                tusb_cfg.phy.self_powered = false;
                tusb_cfg.phy.vbus_monitor_io = -1;
                tusb_cfg.task.size = 4096;
                tusb_cfg.task.priority = 5;
                tusb_cfg.task.xCoreID = 1;
                esp_err_t err = tinyusb_driver_install(&tusb_cfg);
                esp_rom_printf("USB: cdc driver_install=0x%x\n", err);
                if (err == ESP_OK) {
                    s_tusb_installed = true;
                }
            }

#if defined(CONFIG_TINYUSB_CDC_ENABLED)
            if (s_tusb_installed && !s_cdc_acm_inited) {
                tinyusb_config_cdcacm_t acm_cfg = {};
                acm_cfg.cdc_port = TINYUSB_CDC_ACM_0;
                esp_err_t err = tinyusb_cdcacm_init(&acm_cfg);
                esp_rom_printf("USB: cdcacm_init=0x%x\n", err);
                if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
                    s_cdc_acm_inited = tinyusb_cdcacm_initialized(TINYUSB_CDC_ACM_0);
                }
            }

            cdcActive = (s_tusb_installed && s_cdc_acm_inited);
            startOk = cdcActive;
#else
            esp_rom_printf("USB: CDC not enabled in sdkconfig\n");
            cdcActive = false;
            startOk = false;
#endif
#else
            cdcActive = true;
            startOk = true;
#endif
            break;
#endif

        default:
            startOk = false;
            break;
    }

    usbDriverStarted = startOk;
    if (!startOk) {
        esp_rom_printf("USB: start failed mode=%d\n", getSelectedUsbMode());
        setSelectedUsbMode(USB_UNSELECTED_MODE);
    }
}

void usbStop()
{
#if defined(ESP_PLATFORM)
    teardownTinyUsbStack();

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
#if defined(ESP_PLATFORM)
    if (!usbStarted() || getSelectedUsbMode() != USB_JOYSTICK_MODE) return;
    if (!tud_hid_ready()) return;

    usbReport_t report = usbReport();
    if (report.ptr && report.size) {
        tud_hid_report(0, report.ptr, report.size);
    }
#endif
}

#if defined(USB_SERIAL)
void usbSerialPutc(void*, uint8_t c)
{
#if defined(ESP_PLATFORM) && defined(CONFIG_TINYUSB_CDC_ENABLED)
    if (!cdcActive || !tinyusb_cdcacm_initialized(TINYUSB_CDC_ACM_0)) return;
    if (tinyusb_cdcacm_write_queue_char(TINYUSB_CDC_ACM_0, (char)c)) {
        (void)tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
    }
#else
    (void)c;
#endif
}

static void usbSerialSendBuffer(void*, const uint8_t* data, uint32_t size)
{
#if defined(ESP_PLATFORM) && defined(CONFIG_TINYUSB_CDC_ENABLED)
    if (!cdcActive || !data || size == 0 || !tinyusb_cdcacm_initialized(TINYUSB_CDC_ACM_0)) return;
    (void)tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, data, size);
    (void)tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, 0);
#else
    (void)data;
    (void)size;
#endif
}

static int usbSerialGetByte(void*, uint8_t* data)
{
#if defined(ESP_PLATFORM) && defined(CONFIG_TINYUSB_CDC_ENABLED)
    if (!cdcActive || !data || !tinyusb_cdcacm_initialized(TINYUSB_CDC_ACM_0)) return 0;
    size_t rx_size = 0;
    if (tinyusb_cdcacm_read(TINYUSB_CDC_ACM_0, data, 1, &rx_size) == ESP_OK && rx_size == 1) {
        return 1;
    }
#else
    (void)data;
#endif
    return 0;
}

static void* usbSerialInit(void*, const etx_serial_init*)
{
    cdcActive = false;
    return (void*)1;
}

static const etx_serial_driver_t usbSerialDriver = {
    .init = usbSerialInit,
    .deinit = nullptr,
    .sendByte = usbSerialPutc,
    .sendBuffer = usbSerialSendBuffer,
    .waitForTxCompleted = nullptr,
    .getByte = usbSerialGetByte,
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
