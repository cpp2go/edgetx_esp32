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

extern "C" {
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "driver/gpio.h"

/* Littlevgl specific */
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#include "lvgl_helpers.h"
}

static void lcd_flush(lv_disp_drv_t *drv, uint16_t*color_map, const rect_t& rect) {
    lv_area_t area = {
        .x1 = (lv_coord_t)rect.left(), 
        .y1 = (lv_coord_t)rect.top(), 
        .x2 = (lv_coord_t)rect.right(), 
        .y2 = (lv_coord_t)rect.bottom(),
    };
    disp_driver_flush(drv, &area, (lv_color_t *)color_map);
}

void lcdInit()
{
    lvgl_driver_init();
    lcdSetFlushCb(lcd_flush);
}

static TouchState internalTouchState = {0};
struct TouchState getInternalTouchState() {
    return internalTouchState;
}

struct TouchState touchPanelRead() {
    lv_indev_data_t data = {0};
    touch_driver_read(NULL, &data);
    internalTouchState.x = data.point.x;
    internalTouchState.y = data.point.y;
    if (data.state == LV_INDEV_STATE_PRESSED) {
        internalTouchState.event = TE_DOWN;
    } else {
        internalTouchState.event = TE_UP;
    }
    return internalTouchState;
}

bool touchPanelEventOccured() {
    bool ret = (0 == gpio_get_level(TOUCH_IRQ));
    if (!ret) {
        if (internalTouchState.event != TE_UP) {
              internalTouchState.event = TE_UP;
              ret = true; // generate another event for release
        }
    }
    return ret;
}

bool touchPanelInit(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << (int)TOUCH_IRQ),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    return true;
}

void DMAWait()
{
}

void DMACopyBitmap(uint16_t *dest, uint16_t destw, uint16_t desth, uint16_t x,
                   uint16_t y, const uint16_t *src, uint16_t srcw,
                   uint16_t srch, uint16_t srcx, uint16_t srcy, uint16_t w,
                   uint16_t h)
{
    for (int i = 0; i < h; i++) {
        memcpy(dest + (y + i) * destw + x, src + (srcy + i) * srcw + srcx, 2 * w);
    }
}

// 'src' has ARGB4444
// 'dest' has RGB565
// Source/dest both live in PSRAM (the LCD frame buffer is __SDRAM-backed, see
// lcd.cpp). The original loop re-read *p three times per pixel (once per RGB
// channel) straight from PSRAM, which is the real cost here. Compact to one
// read of *p into a register, one write of *p out, and reuse the loaded bg
// value for all three channels. Output is bit-for-bit identical to the prior
// implementation; only the number of PSRAM accesses per pixel dropped 3:1.
void DMACopyAlphaBitmap(uint16_t *dest, uint16_t destw, uint16_t desth,
                        uint16_t x, uint16_t y, const uint16_t *src,
                        uint16_t srcw, uint16_t srch, uint16_t srcx,
                        uint16_t srcy, uint16_t w, uint16_t h)
{
    for (coord_t line = 0; line < h; line++) {
        uint16_t *p = dest + (y + line) * destw + x;
        const uint16_t *q = src + (srcy + line) * srcw + srcx;
        for (coord_t col = 0; col < w; col++) {
            uint16_t sp = *q;            // ARGB4444 source pixel
            uint8_t alpha = sp >> 12;    // 4-bit alpha (0..15)
            uint8_t ialpha = 0x0f - alpha;
            uint16_t dp = *p;            // RGB565 dest pixel (read once)
            uint8_t sr = ((sp >> 8) & 0x0f) << 1;   // src red  -> 5-bit scale
            uint8_t sg = ((sp >> 4) & 0x0f) << 2;   // src green -> 6-bit scale
            uint8_t sb = (sp & 0x0f) << 1;          // src blue  -> 5-bit scale
            uint8_t dr = dp >> 11;                   // dest red   (5-bit)
            uint8_t dg = (dp >> 5) & 0x3f;           // dest green (6-bit)
            uint8_t db = dp & 0x1f;                  // dest blue  (5-bit)
            uint8_t red   = (sr * alpha + dr * ialpha) / 0x0f;
            uint8_t green = (sg * alpha + dg * ialpha) / 0x0f;
            uint8_t blue  = (sb * alpha + db * ialpha) / 0x0f;
            *p = (red << 11) + (green << 5) + (blue << 0);
            p++;
            q++;
        }
    }
}

// 'src' has A8/L8?
// 'dest' has RGB565
// Dest frame buffer lives in PSRAM; read *p once into a register and reuse for
// all three channels instead of re-reading PSRAM per channel. Output identical
// to the prior implementation.
void DMACopyAlphaMask(uint16_t *dest, uint16_t destw, uint16_t desth,
                      uint16_t x, uint16_t y, const uint8_t *src, uint16_t srcw,
                      uint16_t srch, uint16_t srcx, uint16_t srcy, uint16_t w,
                      uint16_t h, uint16_t fg_color)
{
    RGB_SPLIT(fg_color, red, green, blue);

    for (coord_t line = 0; line < h; line++) {
        uint16_t *p = dest + (y + line) * destw + x;
        const uint8_t *q = src + (srcy + line) * srcw + srcx;
        for (coord_t col = 0; col < w; col++) {
            uint16_t opacity = *q >> 4;  // convert to 4 bits (stored in 8bit for DMA)
            uint8_t bgWeight = OPACITY_MAX - opacity;
            uint16_t dp = *p;            // RGB565 dest pixel (read once from PSRAM)
            RGB_SPLIT(dp, bgRed, bgGreen, bgBlue);
            uint16_t r = (bgRed * bgWeight + red * opacity) / OPACITY_MAX;
            uint16_t g = (bgGreen * bgWeight + green * opacity) / OPACITY_MAX;
            uint16_t b = (bgBlue * bgWeight + blue * opacity) / OPACITY_MAX;
            *p = RGB_JOIN(r, g, b);
            p++;
            q++;
        }
    }
}

