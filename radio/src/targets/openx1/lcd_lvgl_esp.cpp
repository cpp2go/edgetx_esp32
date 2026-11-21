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
#include "fonts.h"
#include "targets/openx1/diag_stats.h"

#if defined(UI_FRAME_PROFILER)
#include "ui_profiler.h"
#include "esp32_uart_driver.h"
#endif

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
#include "esp_timer.h"

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
#if defined(UI_FRAME_PROFILER)
    uiProfilerEnter(UI_PROF_FLUSH);
#endif
    diagEnter(DIAG_LCD_FLUSH);
    diagLcdArea(rect.w, rect.h);
    if (rect.x == 0 && rect.y == 0 && rect.w == LCD_W && rect.h == LCD_H) {
        // Whole screen: let the panel driver stream the frame buffer directly.
        disp_driver_flush(drv, &area, (lv_color_t *)color_map);
    } else {
        // Partial update: color_map is the frame buffer origin, so only this
        // band is clocked out (rows are strided by the screen width).
        disp_mcu_flush_region(color_map, rect.left(), rect.top(),
                              rect.right(), rect.bottom());
        lv_disp_flush_ready(drv);
    }
    diagExit(DIAG_LCD_FLUSH);
#if defined(UI_FRAME_PROFILER)
    uiProfilerExit(UI_PROF_FLUSH);
#endif
}

// LVGL calls this while a previous flush is still marked busy. Yield so audio
// / other CPU1 work can run instead of spinning the menus task.
static void lcd_wait(lv_disp_drv_t *drv)
{
    (void)drv;
    taskYIELD();
}

void lcdInit()
{
    // touchPanelInit() configures the touch IRQ pin (input + pull-up). It was
    // never called before, so touchPanelEventOccured() was sampling a pin that
    // had not been configured and the whole touch path depended on luck.
    touchPanelInit();
    lvgl_driver_init();
    lcdSetFlushCb(lcd_flush);
    lcdSetWaitCb(lcd_wait);
}

#if defined(UI_FRAME_PROFILER)

// UI frame timing overlay. Everything is accumulated in microseconds and
// averaged over a 1s window, then rendered into a label on the LVGL system
// layer so it stays on top of the fullscreen Lua widgets.
static int64_t profStageStart[UI_PROF_STAGE_COUNT];
static uint32_t profStageTotal[UI_PROF_STAGE_COUNT];
static uint32_t profStageCount[UI_PROF_STAGE_COUNT];
static int64_t profLastFrame;
static int64_t profWindowStart;
static uint32_t profFrames;
static uint32_t profPeriodTotal;
static uint32_t profLastUartEvents;
static uint32_t profLastUartBytes;
static lv_obj_t* profLabel;

void uiProfilerEnter(unsigned int stage)
{
    if (stage < UI_PROF_STAGE_COUNT) profStageStart[stage] = esp_timer_get_time();
}

void uiProfilerExit(unsigned int stage)
{
    if (stage >= UI_PROF_STAGE_COUNT) return;
    profStageTotal[stage] += (uint32_t)(esp_timer_get_time() - profStageStart[stage]);
}

// Count calls per frame (used to detect a widget being redrawn more than once
// per LVGL refresh), without touching the accumulated time.
void uiProfilerCount(unsigned int stage)
{
    if (stage < UI_PROF_STAGE_COUNT) profStageCount[stage] += 1;
}

// Lua budget usage reported by the widget refresh() call: 1 unit == 400 VM
// instructions, 100 == the per-frame limit. Captured inside refresh() because
// LuaWidget::update() resets it again afterwards.
static uint32_t profLuaPercent;

void uiProfilerLuaInstructions(unsigned int percent)
{
    profLuaPercent = percent;
}

static int64_t profDrawStart[UI_DRAW_COUNT];
static uint32_t profDrawTotal[UI_DRAW_COUNT];
static uint32_t profDrawCalls[UI_DRAW_COUNT];

void uiProfilerDrawEnter(unsigned int kind)
{
    if (kind < UI_DRAW_COUNT) profDrawStart[kind] = esp_timer_get_time();
}

void uiProfilerDrawExit(unsigned int kind)
{
    if (kind >= UI_DRAW_COUNT) return;
    profDrawTotal[kind] += (uint32_t)(esp_timer_get_time() - profDrawStart[kind]);
    profDrawCalls[kind] += 1;
}

// 1/10 ms resolution, integer only (no float printf on the render path)
static void profTenth(uint32_t us, uint32_t& whole, uint32_t& tenth)
{
    whole = us / 1000;
    tenth = (us % 1000) / 100;
}

static void uiProfilerRender()
{
    if (!profLabel) {
        profLabel = lv_label_create(lv_layer_sys());
        if (!profLabel) return;
        lv_obj_set_style_bg_opa(profLabel, LV_OPA_70, 0);
        lv_obj_set_style_bg_color(profLabel, lv_color_black(), 0);
        lv_obj_set_style_text_color(profLabel, lv_color_white(), 0);
        lv_obj_set_style_text_font(profLabel, getFont(FONT(XXS)), 0);
        lv_obj_set_style_pad_all(profLabel, 2, 0);
        lv_obj_set_pos(profLabel, 0, 0);
        lv_obj_add_flag(profLabel, LV_OBJ_FLAG_IGNORE_LAYOUT);
        lv_obj_clear_flag(profLabel, LV_OBJ_FLAG_CLICKABLE);
    }

    const uint32_t f = profFrames ? profFrames : 1;
    uint32_t periodUs = profPeriodTotal / f;
    uint32_t fps = periodUs ? (1000000 + periodUs / 2) / periodUs : 0;

    uint32_t lvglW, lvglT, uiW, uiT, luaW, luaT, flW, flT, drawW, drawT;
    profTenth(profStageTotal[UI_PROF_LVGL] / f, lvglW, lvglT);
    profTenth(profStageTotal[UI_PROF_UI] / f, uiW, uiT);
    profTenth(profStageTotal[UI_PROF_LUA] / f, luaW, luaT);
    profTenth(profStageTotal[UI_PROF_FLUSH] / f, flW, flT);
    profTenth(profStageTotal[UI_PROF_LUA_DRAW] / f, drawW, drawT);

    uint32_t txtW, txtT, rctW, rctT, imgW, imgT, shpW, shpT, linW, linT;
    profTenth(profDrawTotal[UI_DRAW_TEXT] / f, txtW, txtT);
    profTenth(profDrawTotal[UI_DRAW_RECT] / f, rctW, rctT);
    profTenth(profDrawTotal[UI_DRAW_IMAGE] / f, imgW, imgT);
    profTenth(profDrawTotal[UI_DRAW_SHAPE] / f, shpW, shpT);
    profTenth(profDrawTotal[UI_DRAW_LINE] / f, linW, linT);

    // Telemetry-side diagnostic: RX traffic on the external module UART.
    uint32_t uartEv = 0, uartBytes = 0;
    espUartGetRxStats(EXTMOD_UART_PORT, &uartEv, &uartBytes);
    uint32_t dEv = uartEv - profLastUartEvents;
    uint32_t dBytes = uartBytes - profLastUartBytes;
    profLastUartEvents = uartEv;
    profLastUartBytes = uartBytes;

    lv_label_set_text_fmt(profLabel,
                          "FPS %u  tick %u.%ums\n"
                          "LVGL %u.%u  FL %u.%u\n"
                          "DRAW %u.%u x%u\n"
                          "TXT %u.%u n%u  RCT %u.%u\n"
                          "IMG %u.%u n%u  SHP %u.%u\n"
                          "LIN %u.%u  UI %u.%u  L%% %u\n"
                          "EXT ev%u by%u",
                          (unsigned)fps, (unsigned)(periodUs / 1000),
                          (unsigned)((periodUs % 1000) / 100),
                          (unsigned)lvglW, (unsigned)lvglT, (unsigned)flW,
                          (unsigned)flT, (unsigned)drawW, (unsigned)drawT,
                          (unsigned)(profStageCount[UI_PROF_LUA_DRAW] / f),
                          (unsigned)txtW, (unsigned)txtT,
                          (unsigned)(profDrawCalls[UI_DRAW_TEXT] / f),
                          (unsigned)rctW, (unsigned)rctT, (unsigned)imgW,
                          (unsigned)imgT,
                          (unsigned)(profDrawCalls[UI_DRAW_IMAGE] / f),
                          (unsigned)shpW, (unsigned)shpT, (unsigned)linW,
                          (unsigned)linT, (unsigned)uiW, (unsigned)uiT,
                          (unsigned)profLuaPercent, (unsigned)dEv,
                          (unsigned)dBytes);
}

void uiProfilerFrameEnd()
{
    int64_t now = esp_timer_get_time();
    if (!profWindowStart) {
        profWindowStart = now;
        profLastFrame = now;
        return;
    }

    profPeriodTotal += (uint32_t)(now - profLastFrame);
    profLastFrame = now;
    profFrames++;

    if (now - profWindowStart < 1000000) return;

    uiProfilerRender();

    profWindowStart = now;
    profFrames = 0;
    profPeriodTotal = 0;
    for (int i = 0; i < UI_PROF_STAGE_COUNT; i += 1) {
        profStageTotal[i] = 0;
        profStageCount[i] = 0;
    }
    for (int i = 0; i < UI_DRAW_COUNT; i += 1) {
        profDrawTotal[i] = 0;
        profDrawCalls[i] = 0;
    }
}

#endif  // UI_FRAME_PROFILER

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

// Exact floor(t / 15) for t up to 1890 (largest value the alpha blends below
// can produce). Xtensa has no integer divide, so the compiler would otherwise
// call libgcc once per channel per pixel.
static inline uint8_t div15(uint32_t t)
{
    return (uint8_t)((t * 4370u) >> 16);
}

// 'src' has ARGB4444
// 'dest' has RGB565
// Source/dest both live in PSRAM (the LCD frame buffer is __SDRAM-backed, see
// lcd.cpp). Uses one fixed-point multiply-shift instead of an integer divide
// per channel (Xtensa has no hardware divide) and short-circuits fully
// transparent / fully opaque pixels, which is the common case for icon
// backgrounds and glyph bodies. Output is bit-identical to the old code.
void DMACopyAlphaBitmap(uint16_t *dest, uint16_t destw, uint16_t desth,
                        uint16_t x, uint16_t y, const uint16_t *src,
                        uint16_t srcw, uint16_t srch, uint16_t srcx,
                        uint16_t srcy, uint16_t w, uint16_t h)
{
    for (coord_t line = 0; line < h; line++) {
        uint16_t *p = dest + (y + line) * destw + x;
        const uint16_t *q = src + (srcy + line) * srcw + srcx;
        for (coord_t col = 0; col < w; col++) {
            uint16_t sp = *q++;          // ARGB4444 source pixel
            uint8_t alpha = sp >> 12;    // 4-bit alpha (0..15)

            uint8_t sr = ((sp >> 8) & 0x0f) << 1;   // src red  -> 5-bit scale
            uint8_t sg = ((sp >> 4) & 0x0f) << 2;   // src green -> 6-bit scale
            uint8_t sb = (sp & 0x0f) << 1;          // src blue  -> 5-bit scale

            if (alpha == 0) {                        // keep destination
                p++;
                continue;
            }
            if (alpha == OPACITY_MAX) {              // opaque: pure source
                *p++ = (uint16_t)((sr << 11) + (sg << 5) + sb);
                continue;
            }

            uint8_t ialpha = OPACITY_MAX - alpha;
            uint16_t dp = *p;                        // RGB565 dest (read once)
            uint8_t dr = dp >> 11;                   // dest red   (5-bit)
            uint8_t dg = (dp >> 5) & 0x3f;           // dest green (6-bit)
            uint8_t db = dp & 0x1f;                  // dest blue  (5-bit)
            uint8_t red   = div15(sr * alpha + dr * ialpha);
            uint8_t green = div15(sg * alpha + dg * ialpha);
            uint8_t blue  = div15(sb * alpha + db * ialpha);
            *p++ = (uint16_t)((red << 11) + (green << 5) + (blue << 0));
        }
    }
}

// 'src' has A8/L8?
// 'dest' has RGB565
// Dest frame buffer lives in PSRAM; read *p once into a register and reuse for
// all three channels. Fixed-point blending (no integer divide) plus
// transparent/opaque short-circuits; output bit-identical to the old code.
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
            uint8_t opacity = *q++ >> 4;  // convert to 4 bits (stored in 8bit for DMA)
            if (opacity == 0) {            // keep destination
                p++;
                continue;
            }
            if (opacity == OPACITY_MAX) {  // opaque: pure foreground
                *p++ = fg_color;
                continue;
            }
            uint8_t bgWeight = OPACITY_MAX - opacity;
            uint16_t dp = *p;            // RGB565 dest pixel (read once from PSRAM)
            RGB_SPLIT(dp, bgRed, bgGreen, bgBlue);
            uint16_t r = div15(bgRed * bgWeight + red * opacity);
            uint16_t g = div15(bgGreen * bgWeight + green * opacity);
            uint16_t b = div15(bgBlue * bgWeight + blue * opacity);
            *p++ = RGB_JOIN(r, g, b);
        }
    }
}

