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
#include "mcp_pins.h"

/* Littlevgl specific */
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#include "lvgl_helpers.h"

#include "nvs_flash.h"
/* BLE */
//#include "nimble/nimble_port.h"
//#include "nimble/nimble_port_freertos.h"

#include "driver/i2c_master.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

extern void ads1015_adc_init(void);

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

// ---- ES8311 audio codec (OSPTEK P4C5 dev board) ----------------------------
// ES8311 sits on the same I2C0 bus as the touch controller (GPIO7/8, 7-bit
// address 0x18) and receives I2S from the P4 (DOUT=GPIO9 BCLK=GPIO12
// LRCLK=GPIO10 MCLK=GPIO13). The NS4150 power amp enable is GPIO53.
// Register sequence follows the Espressif es8311 driver (MCLK =
// 256 * 32000 Hz = 8.192 MHz, I2S slave, 16-bit, DAC -> speaker/amp).
#if defined(I2S_AMP_EN_GPIO)
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#define ES8311_I2C_ADDR  0x18   /* 7-bit address (0x30 with CE=0 as 8-bit) */

static i2c_master_dev_handle_t es8311_dev = NULL;

static void es8311_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    i2c_master_transmit(es8311_dev, buf, sizeof(buf), 100);
}

static uint8_t es8311_read_reg(uint8_t reg)
{
    uint8_t val = 0;
    if (i2c_master_transmit_receive(es8311_dev, &reg, 1, &val, 1, 100) != ESP_OK) {
        ESP_LOGE("ES8311", "read reg 0x%02x failed", reg);
    }
    return val;
}

static void es8311AudioInit(void)
{
    i2c_device_config_t dev_cfg = {
        .device_address = ES8311_I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    if (i2c_master_bus_add_device(i2c_0_bus_handle, &dev_cfg, &es8311_dev) != ESP_OK) {
        ESP_LOGE("ES8311", "failed to add I2C device");
        return;
    }
    ESP_LOGI("ES8311", "ES8311 codec found on I2C0 @0x%02x", ES8311_I2C_ADDR);

    // Enable the NS4150 class-D power amplifier.
    gpio_config_t pa_cfg = {
        .pin_bit_mask = 1ULL << I2S_AMP_EN_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&pa_cfg);
    gpio_set_level(I2S_AMP_EN_GPIO, 1);

    // --- base init (Espressif es8311 driver) ---
    es8311_write_reg(0x01, 0x30);   // clock manager 1
    es8311_write_reg(0x02, 0x00);   // clock manager 2
    es8311_write_reg(0x03, 0x10);   // clock manager 3 (adc osr)
    es8311_write_reg(0x16, 0x24);   // ADC
    es8311_write_reg(0x04, 0x10);   // dac osr
    es8311_write_reg(0x05, 0x00);   // adc/dac clk divider
    es8311_write_reg(0x0B, 0x00);
    es8311_write_reg(0x0C, 0x00);
    es8311_write_reg(0x10, 0x1F);
    es8311_write_reg(0x11, 0x7F);
    es8311_write_reg(0x00, 0x80);   // reset digital/core/clock
    vTaskDelay(pdMS_TO_TICKS(20));

    // I2S slave mode (P4 is the I2S master), MCLK from the MCLK pin.
    es8311_write_reg(0x00, es8311_read_reg(0x00) & 0xBF);
    es8311_write_reg(0x01, 0x3F);
    es8311_write_reg(0x01, es8311_read_reg(0x01) & 0x7F);  // MCLK source = pin

    // --- clock coefficients for fs = 32 kHz, MCLK = 8.192 MHz ---
    // coeff row {8192000, 32000, pre_div=1, pre_multi=1, adc_div=1, dac_div=1,
    //            fs_mode=0, lrck_h=0, lrck_l=0xff, bclk_div=4, osr=0x10}
    es8311_write_reg(0x02, 0x00);   // (pre_div-1)<<5 | (mult<<3), mult=1 -> 0
    es8311_write_reg(0x03, 0x10);   // fs_mode<<6 | adc_osr
    es8311_write_reg(0x04, 0x10);   // dac_osr
    es8311_write_reg(0x05, 0x00);   // (adc_div-1)<<4 | (dac_div-1)
    es8311_write_reg(0x06, 0x03);   // bclk_div - 1 (4-1)
    es8311_write_reg(0x07, 0x00);   // lrck divider high
    es8311_write_reg(0x08, 0xFF);   // lrck divider low

    // --- serial audio port: I2S, 16-bit, enabled ---
    uint8_t iface = es8311_read_reg(0x09) & 0xBF;   // DAC SDPIN
    iface = (iface & 0xFC) | 0x0C;                  // I2S format + 16-bit
    es8311_write_reg(0x09, iface);
    uint8_t iface_adc = es8311_read_reg(0x0A) & 0xBF;  // ADC SDPOUT
    iface_adc = (iface_adc & 0xFC) | 0x0C;
    es8311_write_reg(0x0A, iface_adc);

    es8311_write_reg(0x13, 0x10);
    es8311_write_reg(0x1B, 0x0A);
    es8311_write_reg(0x1C, 0x6A);

    // --- power up DAC path to speaker / amp ---
    es8311_write_reg(0x32, 0xBF);   // DAC volume ~0 dB (software volume)
    es8311_write_reg(0x37, 0x48);   // DAC ramp rate
    es8311_write_reg(0x17, 0xBF);
    es8311_write_reg(0x0E, 0x02);
    es8311_write_reg(0x12, 0x00);
    es8311_write_reg(0x14, 0x1A);
    es8311_write_reg(0x0D, 0x01);
    es8311_write_reg(0x15, 0x40);
    es8311_write_reg(0x45, 0x00);

    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_LOGI("ES8311", "codec initialized (I2S slave 16-bit, DAC -> speaker)");
}
#endif  // I2S_AMP_EN_GPIO

// keep a reference of the layouts so they do not get optimized out by compiler.
#if 1
#include "layout.h"

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
    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if  (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 蓝牙协议初始化
    nimble_port_init();

    board_init_i2c();

    keysInit();
    rtcInit();

    sdInit();

#if defined(ROTARY_ENCODER_NAVIGATION)
    rotaryEncoderInit();
#endif

    //backlightInit();
    initWiFi();
    init2MhzTimer();
    
    audioInit();
#if defined(I2S_AMP_EN_GPIO)
    es8311AudioInit();   // ES8311 codec + NS4150 amp (OSPTEK P4C5 dev board)
#endif
    ads1015_adc_init();
}

void boardOff()
{
    lcdFadeOut();
    pwrOff();
}

extern uint32_t ShadowInput;

int usbPlugged() {
    // OpenX1 reports VBUS via MCP23017 G1B5 (see mcp_pins.h)
    // Debounce the raw input so a glitchy VBUS doesn't toggle USB on/off
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
void DMAInit(void)
{
    
}