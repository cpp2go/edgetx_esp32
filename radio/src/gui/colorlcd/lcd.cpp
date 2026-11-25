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

#include "lcd.h"

#include <lvgl/lvgl.h>

#include "bitmapbuffer.h"
#include "board.h"
#include "etx_lv_theme.h"
#if !LV_USE_GPU_STM32_DMA2D && !defined(SIMU)
#include "dma2d.h"
#endif

#if LV_MEM_CUSTOM == 0
char LVGL_MEM_BUFFER[LV_MEM_SIZE] __SDRAM __ALIGNED(16);

char* get_lvgl_mem(int nbytes)
{
  UNUSED(nbytes);
  return LVGL_MEM_BUFFER;
}
#endif

pixel_t LCD_FIRST_FRAME_BUFFER[DISPLAY_BUFFER_SIZE] __SDRAM;
#if defined(ESP32_SINGLE_FRAMEBUFFER)
pixel_t* LCD_SECOND_FRAME_BUFFER = LCD_FIRST_FRAME_BUFFER;
#else
pixel_t LCD_SECOND_FRAME_BUFFER[DISPLAY_BUFFER_SIZE] __SDRAM;
#endif

BitmapBuffer lcdBuffer1(BMP_RGB565, LCD_W, LCD_H,
                        (uint16_t*)LCD_FIRST_FRAME_BUFFER);
BitmapBuffer lcdBuffer2(BMP_RGB565, LCD_W, LCD_H,
                        (uint16_t*)LCD_SECOND_FRAME_BUFFER);

static BitmapBuffer* lcdFront = &lcdBuffer1;
static BitmapBuffer* lcd = &lcdBuffer2;

static lv_disp_draw_buf_t disp_buf;
static lv_disp_drv_t disp_drv;

// Call backs
static void (*lcd_wait_cb)(lv_disp_drv_t*) = nullptr;
static void (*lcd_flush_cb)(lv_disp_drv_t*, uint16_t* buffer,
                            const rect_t& area) = nullptr;

void lcdSetWaitCb(void (*cb)(lv_disp_drv_t*)) { lcd_wait_cb = cb; }

void lcdSetFlushCb(void (*cb)(lv_disp_drv_t*, uint16_t*, const rect_t&))
{
  lcd_flush_cb = cb;
}

static lv_disp_drv_t* refr_disp = nullptr;

// Compile-time A/B switch for the panel refresh path:
//   1 = push only the areas LVGL actually redrew (less panel traffic)
//   0 = push the whole screen once per frame (the behaviour before that
//       optimisation, simplest possible)
// Used to A/B a tearing / flicker report on real hardware.
#ifndef LCD_FLUSH_DIRTY_AREAS
#define LCD_FLUSH_DIRTY_AREAS 1
#endif

#if defined(ESP_PLATFORM)
// In direct mode LVGL draws each invalid area in place and then calls the flush
// callback with the *whole screen* (refr_area() in lv_refr.c), so the flush
// callback alone cannot tell what changed. The invalid areas themselves are
// still available while the refresh runs, so on the last call of a frame push
// only those: a key press then costs a few hundred microseconds of panel bus
// instead of a full screen (12ms), which is both faster and tear-free.
static void pushInvalidAreas(lv_disp_drv_t* disp_drv, uint16_t* color_p)
{
#if !LCD_FLUSH_DIRTY_AREAS
  rect_t full = {0, 0, LCD_W, LCD_H};
  lcd_flush_cb(disp_drv, color_p, full);
  return;
#else
  auto disp = _lv_refr_get_disp_refreshing();
  if (!disp) {
    rect_t full = {0, 0, LCD_W, LCD_H};
    lcd_flush_cb(disp_drv, color_p, full);
    return;
  }

  uint32_t total = 0;
  int count = 0;
  for (uint16_t i = 0; i < disp->inv_p; i++) {
    if (disp->inv_area_joined[i]) continue;
    total += lv_area_get_size(&disp->inv_areas[i]);
    count++;
  }

  // Nothing (or nearly the whole screen) changed: the single full-screen
  // transfer is the cheapest option.
  if (count == 0 || total * 5 > (uint32_t)LCD_W * LCD_H * 3) {
    rect_t full = {0, 0, LCD_W, LCD_H};
    lcd_flush_cb(disp_drv, color_p, full);
    return;
  }

  for (uint16_t i = 0; i < disp->inv_p; i++) {
    if (disp->inv_area_joined[i]) continue;
    const lv_area_t* a = &disp->inv_areas[i];
    rect_t r = {a->x1, a->y1, a->x2 - a->x1 + 1, a->y2 - a->y1 + 1};
    lcd_flush_cb(disp_drv, color_p, r);
  }
#endif  // LCD_FLUSH_DIRTY_AREAS
}
#endif

static void flushLcd(lv_disp_drv_t* disp_drv, const lv_area_t* area,
                     lv_color_t* color_p)
{
  // LVGL direct mode: the frame buffer *is* the screen, so LVGL draws each
  // invalid area in place and calls this callback with buf_area == the whole
  // screen (see refr_area() in lv_refr.c). Pushing every call would send the
  // full screen 2..4 times per frame (measured: 12ms of DMA each) for nothing.
#if !defined(LCD_VERTICAL_INVERT) || defined(RADIO_F16)
#if defined(RADIO_F16)
  if (hardwareOptions.pcbrev > 0)
#endif
  {
    // we're only interested in the last flush in direct mode
    if (!lv_disp_flush_is_last(disp_drv)) {
      lv_disp_flush_ready(disp_drv);
      return;
    }
  }
#endif

#if defined(DEBUG_WINDOWS)
  if (area->x1 != 0 || area->x2 != LCD_W - 1 || area->y1 != 0 ||
      area->y2 != LCD_H - 1) {
    TRACE("partial refresh @ 0x%p {%d,%d,%d,%d}", color_p, area->x1, area->y1,
          area->x2, area->y2);
  } else {
    TRACE("full refresh @ 0x%p", color_p);
  }
#endif

  if (lcd_flush_cb) {
    refr_disp = disp_drv;

#if defined(ESP_PLATFORM)
    pushInvalidAreas(disp_drv, (uint16_t*)color_p);
    // The ESP I80 flush callback blocks until the pixels are on the wire and
    // calls lv_disp_flush_ready() itself (disp_mcu.c).
    return;
#else
    rect_t copy_area = {area->x1, area->y1, area->x2 - area->x1 + 1,
                        area->y2 - area->y1 + 1};

    lcd_flush_cb(disp_drv, (uint16_t*)color_p, copy_area);
#endif
  }

  lv_disp_flush_ready(disp_drv);
}

static void clear_frame_buffers()
{
  memset(LCD_FIRST_FRAME_BUFFER, 0, sizeof(LCD_FIRST_FRAME_BUFFER));
#if !defined(ESP32_SINGLE_FRAMEBUFFER)
  memset(LCD_SECOND_FRAME_BUFFER, 0, sizeof(LCD_SECOND_FRAME_BUFFER));
#endif
}

static void init_lvgl_disp_drv()
{
  lv_disp_draw_buf_init(&disp_buf, lcdFront->getData(),
#if defined(ESP32_SINGLE_FRAMEBUFFER)
                        NULL,
#else
                        lcd->getData(),
#endif
                        LCD_W * LCD_H);
  lv_disp_drv_init(&disp_drv); /*Basic initialization*/

  disp_drv.draw_buf = &disp_buf; /*Set an initialized buffer*/
  disp_drv.flush_cb = flushLcd;  /*Set a flush callback to draw to the display*/
  disp_drv.wait_cb = lcd_wait_cb; /*Set a wait callback*/

  disp_drv.hor_res = LCD_W; /*Set the horizontal resolution in pixels*/
  disp_drv.ver_res = LCD_H; /*Set the vertical resolution in pixels*/
  disp_drv.full_refresh = 0;

#if !defined(LCD_VERTICAL_INVERT)
  disp_drv.direct_mode = 1;
#elif defined(RADIO_F16)
  disp_drv.direct_mode = (hardwareOptions.pcbrev > 0) ? 1 : 0;
#else
  disp_drv.direct_mode = 0;
#endif
}

void lcdInitDisplayDriver()
{
  static bool lcdDriverStarted = false;
  // we already have a display: exit
  if (lcdDriverStarted) return;
  lcdDriverStarted = true;

#if !LV_USE_GPU_STM32_DMA2D && !defined(SIMU)
  DMAInit();
#endif

  // Full LVGL init in firmware mode
  lv_init();
  // Initialise styles
  useMainStyle();

  // Clear buffers first
  clear_frame_buffers();
  lcdSetInitalFrameBuffer(lcdFront->getData());

  // Init hardware LCD driver
  lcdInit();
  backlightInit();

  init_lvgl_disp_drv();

  // Register the driver and save the created display object
  lv_disp_drv_register(&disp_drv);

  // remove all styles on default screen (makes it transparent as well)
  lv_obj_remove_style_all(lv_scr_act());
}

void lcdFlushed()
{
  // Only used in simulator

  // its possible to get here before flushLcd is ever called.
  // so check for nullptr first. (Race condition if you put breakpoints in
  // startup code)
  if (refr_disp != nullptr) lv_disp_flush_ready(refr_disp);
}

#if defined(SIMU)
void lcdRefresh() {}
#endif
