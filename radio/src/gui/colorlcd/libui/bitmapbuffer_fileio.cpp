/*
 * Copyright (C) EdgeTX
 *
 * Based on code named
 *   libopenui - https://github.com/opentx/libopenui
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

#pragma GCC optimize("O3")

#include "bitmapbuffer.h"
#include "lib_file.h"
#include "edgetx_helpers.h"

#if defined(ESP_PLATFORM)
#include "esp_heap_caps.h"
#endif

FIL imgFile __DMA;

// #define TRACE_STB_MALLOC

#if defined(TRACE_STB_MALLOC)
#define STBI_MALLOC(sz) stb_malloc(sz)
#define STBI_REALLOC_SIZED(p, oldsz, newsz) stb_realloc(p, oldsz, newsz)
#define STBI_FREE(p) stb_free(p)

void *stb_malloc(unsigned int size)
{
  void *res = malloc(size);
  TRACE("malloc %d = %p", size, res);
  return res;
}

void stb_free(void *ptr)
{
  TRACE("free %p", ptr);
  free(ptr);
}

void *stb_realloc(void *ptr, unsigned int oldsz, unsigned int newsz)
{
  void *res = realloc(ptr, newsz);
  TRACE("realloc %p, %d -> %d = %p", ptr, oldsz, newsz, res);
  return res;
}
#endif  // #if defined(TRACE_STB_MALLOC)

#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#undef __I
#define STBI_ONLY_PNG
#define STBI_ONLY_BMP
#define STBI_ONLY_JPEG
#define STBI_NO_STDIO
#define STBI_NO_LINEAR
#define STB_IMAGE_IMPLEMENTATION
#define STBI_NO_THREAD_LOCALS
#include "stb/stb_image.h"

// fill 'data' with 'size' bytes.  return number of bytes actually read
int stbc_read(void *user, char *data, int size)
{
  FIL *fp = (FIL *)user;
  UINT br = 0;
  FRESULT res = f_read(fp, data, size, &br);
  if (res == FR_OK) {
    return (int)br;
  }
  return 0;
}

// skip the next 'n' bytes, or 'unget' the last -n bytes if negative
void stbc_skip(void *user, int n)
{
  FIL *fp = (FIL *)user;
  f_lseek(fp, f_tell(fp) + n);
}

// returns nonzero if we are at end of file/data
int stbc_eof(void *user)
{
  FIL *fp = (FIL *)user;
  int res = f_eof(fp);
  return res;
}

// callbacks for stb-image
const stbi_io_callbacks stbCallbacks = {stbc_read, stbc_skip, stbc_eof};

#if defined(ESP_PLATFORM)
static void trace_heap_caps(const char *tag)
{
  size_t free_spiram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  size_t largest_spiram = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
  size_t free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  size_t largest_8bit = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);

  TRACE("%s heap: PSRAM free=%u largest=%u | 8BIT free=%u largest=%u", tag,
        (unsigned)free_spiram, (unsigned)largest_spiram, (unsigned)free_8bit,
        (unsigned)largest_8bit);
}
#else
static inline void trace_heap_caps(const char *) {}
#endif

static inline void unpack_pixel(const uint8_t *p, int n, uint8_t &r,
                                uint8_t &g, uint8_t &b, uint8_t &a)
{
  switch (n) {
    case 1:  // Gray
      r = g = b = p[0];
      a = 0xFF;
      break;
    case 2:  // Gray + Alpha
      r = g = b = p[0];
      a = p[1];
      break;
    case 3:  // RGB
      r = p[0];
      g = p[1];
      b = p[2];
      a = 0xFF;
      break;
    default: // RGBA
      r = p[0];
      g = p[1];
      b = p[2];
      a = p[3];
      break;
  }
}

BitmapBuffer *BitmapBuffer::loadBitmap(const char *filename, BitmapFormats fmt)
{
  FRESULT result = f_open(&imgFile, filename, FA_OPEN_EXISTING | FA_READ);
  if (result != FR_OK) {
    trace_heap_caps("loadBitmap open failed");
    return nullptr;
  }

  int x, y, nn;
  stbi_info_from_callbacks(&stbCallbacks, &imgFile, &x, &y, &nn);
  f_lseek(&imgFile, 0);

    int w, h, n;
    unsigned char *img =
      stbi_load_from_callbacks(&stbCallbacks, &imgFile, &w, &h, &n, 0);
  f_close(&imgFile);

  if (!img) {
    TRACE_ERROR("loadBitmap(%s) failed: %s\n", filename, stbi_failure_reason());
    trace_heap_caps("loadBitmap decode failed");
    return nullptr;
  }

  // convert to RGB565 or ARGB4444 format
  BitmapFormats dst_fmt = fmt;
  if (dst_fmt == BMP_INVALID) {
    dst_fmt = (n == 2 || n == 4 ? BMP_ARGB4444 : BMP_RGB565);
  }

  BitmapBuffer *bmp = new BitmapBuffer(dst_fmt, w, h);
  if (bmp == nullptr || bmp->getData() == nullptr) {
    if (bmp) delete bmp;
    stbi_image_free(img);
    TRACE_ERROR("loadBitmap: malloc failed\n");
    trace_heap_caps("loadBitmap dest alloc failed");
    return nullptr;
  }

  pixel_t *dest = bmp->getPixelPtrAbs(0, 0);
  const uint8_t *p = img;
  int step = (n >= 1 && n <= 4) ? n : 4;
  if (dst_fmt == BMP_ARGB4444) {
    for (int row = 0; row < h; ++row) {
      for (int col = 0; col < w; ++col) {
        uint8_t r, g, b, a;
        unpack_pixel(p, n, r, g, b, a);
        *dest = ARGB(a, r, g, b);
        MOVE_TO_NEXT_RIGHT_PIXEL(dest);
        p += step;
      }
    }
  } else {
    for (int row = 0; row < h; ++row) {
      for (int col = 0; col < w; ++col) {
        uint8_t r, g, b, a;
        unpack_pixel(p, n, r, g, b, a);
        *dest = RGB(r, g, b);
        MOVE_TO_NEXT_RIGHT_PIXEL(dest);
        p += step;
      }
    }
  }

  stbi_image_free(img);
  return bmp;
}

//-----------------------------------------------------------------------------

static lv_res_t decoder_info(struct _lv_img_decoder_t *decoder, const void *src,
                             lv_img_header_t *header)
{
  LV_UNUSED(decoder); /*Unused*/

  lv_img_src_t src_type = lv_img_src_get_type(src); /*Get the source type*/

  /*If it's a file...*/
  if (src_type == LV_IMG_SRC_FILE) {
    const char *fn = ((const char *)src) + 1;
    FIL imgFile;

    FRESULT result = f_open(&imgFile, fn, FA_OPEN_EXISTING | FA_READ);
    if (result == FR_OK) {
      int x, y, nn;
      int res = stbi_info_from_callbacks(&stbCallbacks, &imgFile, &x, &y, &nn);
      f_close(&imgFile);

      if (res == LV_RES_INV) {
        TRACE_ERROR("decoder_info(%s) failed: %s\n", fn, stbi_failure_reason());
        return LV_RES_INV;
      }

      header->always_zero = 0;
      header->cf =
          (nn == 4) ? LV_IMG_CF_TRUE_COLOR_ALPHA : LV_IMG_CF_TRUE_COLOR;
      header->w = x;
      header->h = y;

      return LV_RES_OK;
    } else {
      TRACE_ERROR("decoder_info(%s) failed to open image file\n", fn);
    }
  }
  /*If it's a file in a C array...*/
  else if (src_type == LV_IMG_SRC_VARIABLE) {
    // Not implemented...
  }

  return LV_RES_INV;
}

static uint8_t *convert_bitmap(uint8_t *img, int w, int h, int n)
{
  bool has_alpha = (n == 2 || n == 4);
  uint8_t *bmp = (uint8_t *)lv_mem_alloc((has_alpha ? 3 : 2) * w * h);
  if (bmp == nullptr) {
    TRACE_ERROR("convert_bitmap: lv_mem_alloc failed\n");
    trace_heap_caps("convert_bitmap lv_mem_alloc failed");
    return nullptr;
  }

  const uint8_t *p = img;
  int step = (n >= 1 && n <= 4) ? n : 4;
  if (has_alpha) {
    uint8_t *dest = bmp;
    for (int row = 0; row < h; ++row) {
      for (int col = 0; col < w; ++col) {
        uint8_t r, g, b, a;
        unpack_pixel(p, n, r, g, b, a);
        uint16_t c = RGB(r, g, b);
        *dest++ = c & 0xFF;
        *dest++ = c >> 8;
        *dest++ = a;
        p += step;
      }
    }
  } else {
    pixel_t *dest = (pixel_t *)bmp;
    for (int row = 0; row < h; ++row) {
      for (int col = 0; col < w; ++col) {
        uint8_t r, g, b, a;
        unpack_pixel(p, n, r, g, b, a);
        *dest = RGB(r, g, b);
        MOVE_TO_NEXT_RIGHT_PIXEL(dest);
        p += step;
      }
    }
  }

  return bmp;
}

static lv_res_t decoder_open(lv_img_decoder_t *decoder,
                             lv_img_decoder_dsc_t *dsc)
{
  LV_UNUSED(decoder); /*Unused*/

  /*If it's a file...*/
  if (dsc->src_type == LV_IMG_SRC_FILE) {
    const char *fn = ((const char *)dsc->src) + 1;
    FIL imgFile;

    FRESULT result = f_open(&imgFile, fn, FA_OPEN_EXISTING | FA_READ);
    if (result == FR_OK) {
      int w, h, n;
      unsigned char *img =
          stbi_load_from_callbacks(&stbCallbacks, &imgFile, &w, &h, &n, 0);
      f_close(&imgFile);

      if (!img) {
        TRACE_ERROR("decoder_open(%s) failed: %s\n", fn, stbi_failure_reason());
        trace_heap_caps("decoder_open decode failed");
        return LV_RES_INV;
      }

      dsc->img_data = convert_bitmap(img, w, h, n);
      stbi_image_free(img);

      if (!dsc->img_data) {
        trace_heap_caps("decoder_open convert failed");
      }

      return dsc->img_data ? LV_RES_OK : LV_RES_INV;
    } else {
      TRACE_ERROR("decoder_open(%s) failed to open image file\n", fn);
    }
  }
  /*If it's a file in a C array...*/
  else if (dsc->src_type == LV_IMG_SRC_VARIABLE) {
    // Not implemented...
  }

  return LV_RES_INV;
}

static void decoder_close(lv_img_decoder_t *decoder, lv_img_decoder_dsc_t *dsc)
{
  LV_UNUSED(decoder); /*Unused*/
  if (dsc->img_data) {
    lv_mem_free((uint8_t *)dsc->img_data);
    dsc->img_data = NULL;
  }
}

/**
 * Register the STB decoder functions in LVGL
 */
void lv_stb_init(void)
{
  lv_img_decoder_t *dec = lv_img_decoder_create();
  lv_img_decoder_set_info_cb(dec, decoder_info);
  lv_img_decoder_set_open_cb(dec, decoder_open);
  lv_img_decoder_set_close_cb(dec, decoder_close);
}
