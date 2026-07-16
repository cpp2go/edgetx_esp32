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
#include "hal/usb_driver.h"
#include "hal/adc_driver.h"

/* Littlevgl specific */
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#include "lvgl_helpers.h"

#include "nvs_flash.h"
#include "esp_log.h"
/* BLE */
//#include "nimble/nimble_port.h"
//#include "nimble/nimble_port_freertos.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "flyskyHallStick_driver.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "mcp_pins.h"

extern void ads1015_adc_init(void);
extern uint32_t ShadowInput;

i2c_master_bus_handle_t i2c_0_bus_handle;
i2c_master_bus_handle_t lvgl_i2c_bus_handle;
i2c_master_bus_handle_t rtc_i2c_bus_handle;
i2c_master_bus_handle_t gpioext_i2c_bus_handle;
i2c_master_bus_handle_t toplcd_i2c_bus_handle;
i2c_master_bus_handle_t ads_i2c_bus_handle;
static void board_init_i2c(void) {
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_0_SDA,
        .scl_io_num = I2C_0_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
          .enable_internal_pullup = 1
        }
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_0_bus_handle));
    lvgl_i2c_bus_handle = i2c_0_bus_handle;
    rtc_i2c_bus_handle = i2c_0_bus_handle;
    gpioext_i2c_bus_handle = i2c_0_bus_handle;
    toplcd_i2c_bus_handle = i2c_0_bus_handle;
#if 1
    ads_i2c_bus_handle = i2c_0_bus_handle;
#else
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_1,
        .sda_io_num = I2C_1_SDA,
        .scl_io_num = I2C_1_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
          .enable_internal_pullup = 1
        }
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_1_bus_handle));
    ads_i2c_bus_handle = i2c_1_bus_handle;
#endif
}

// keep a reference of the layouts so they do not get optimized out by compiler.
#if 1
extern LayoutFactory Layout1P2;
extern LayoutFactory Layout1P3;
extern LayoutFactory layout1x1;
extern LayoutFactory Layout1x2;
extern LayoutFactory Layout1x3;
extern LayoutFactory Layout1x4;
extern LayoutFactory layout2P1;
extern LayoutFactory Layout2P3;
extern LayoutFactory Layout2x1;
extern LayoutFactory layout2x2;
extern LayoutFactory layout2x3;
extern LayoutFactory layout2x4;
extern LayoutFactory layout4P2;
LayoutFactory *layouts[] = {
    &Layout1P2, &Layout1P3, &layout1x1, &Layout1x2, &Layout1x3, &Layout1x4,
    &layout2P1, &Layout2P3, &Layout2x1, &layout2x2, &layout2x3, &layout2x4,
    &layout4P2
};

extern WidgetFactory gaugeWidget;
extern WidgetFactory modelBitmapWidget;
extern WidgetFactory outputsWidget;
extern WidgetFactory RadioInfoWidget;
extern WidgetFactory DateTimeWidget;
extern WidgetFactory textWidget;
extern WidgetFactory timerWidget;
extern WidgetFactory ValueWidget;
WidgetFactory *widgets[] = {
    &gaugeWidget, &modelBitmapWidget, &outputsWidget, &RadioInfoWidget, 
	&DateTimeWidget,&textWidget, &timerWidget, &ValueWidget
};
#endif

void boardInit()
{
    ESP_EARLY_LOGI("BOARD", "boardInit start");

    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if  (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_EARLY_LOGI("BOARD", "nvs_flash_init done");

    nimble_port_init();
    ESP_EARLY_LOGI("BOARD", "nimble_port_init done");

    board_init_i2c();
    ESP_EARLY_LOGI("BOARD", "board_init_i2c done");

    keysInit();
    rtcInit();
    ESP_EARLY_LOGI("BOARD", "keysInit+rtcInit done");

    sdInit();
    ESP_EARLY_LOGI("BOARD", "sdInit done");

#if defined(ROTARY_ENCODER_NAVIGATION)
    rotaryEncoderInit();
    ESP_EARLY_LOGI("BOARD", "rotaryEncoderInit done");
#endif

    //backlightInit();
    initWiFi();
    ESP_EARLY_LOGI("BOARD", "initWiFi done");
    init2MhzTimer();

    audioInit();
    ESP_EARLY_LOGI("BOARD", "audioInit done");
    ads1015_adc_init();
    ESP_EARLY_LOGI("BOARD", "ads1015_adc_init done");

    ESP_EARLY_LOGI("BOARD", "boardInit complete");
    // usbInit() / usb_device_task removed: tinyusb_driver_install() (called lazily
    // from usbStart()) creates its own tinyusb_device_task; a duplicate tud_task()
    // loop here would race against it and cause a crash.
}

void boardOff()
{
    lcdFadeOut();
    pwrOff();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

int usbPlugged() {
    static uint8_t debouncedState = 0;
    static uint8_t lastState = 0;

    uint8_t state = (ShadowInput & (1U << USB_GPIO_PIN_VBUS)) ? 1u : 0u;
    if (state == lastState)
        debouncedState = state;
    else
        lastState = state;

    return debouncedState;
}

void enableVBatBridge() {
}
void disableVBatBridge() {
}
bool isVBatBridgeEnabled() {
    return false;
}

uint16_t getBatteryVoltage()
{
    if (adcGetMaxInputs(ADC_INPUT_VBAT) < 1) return 0;
    // ADS1015 GAIN_ONE: FSR=4.096V, 1 LSB=2mV; anaIn() returns raw_12bit/2.
    // Each anaIn() step = 4mV.  Voltage divider: 10k (top) + 3k (bottom),
    // ratio = 3/13 → V_bat = V_adc × 13/3.
    // Result in 0.01V units: val * 4mV * 13/3 / 10 = val * 52 / 30.
    uint16_t val = anaIn(adcGetInputOffset(ADC_INPUT_VBAT));
    int32_t result = (int32_t)val * 52 / 30 + g_eeGeneral.txVoltageCalibration;
    return result > 0 ? (uint16_t)result*2 : 0;
}

uint16_t getRTCBatteryVoltage()
{
    return 0;
}

void pollKeys()
{
}

void lcdSetInitalFrameBuffer(void* fbAddress)
{
}