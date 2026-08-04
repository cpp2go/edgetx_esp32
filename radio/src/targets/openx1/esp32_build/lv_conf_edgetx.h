/**
 * @file lv_conf.h
 * Configuration file for v8.2.0
 */

/*
 * Copy this file as `lv_conf.h`
 * 1. simply next to the `lvgl` folder
 * 2. or any other places and
 *    - define `LV_CONF_INCLUDE_SIMPLE`
 *    - add the path as include path
 */

/* clang-format off */
#if 1 /*Set it to "1" to enable content*/

#ifndef LV_CONF_EDGETX_H
#define LV_CONF_EDGETX_H

#if !defined(CONFIG_LV_TFT_DISPLAY_CONTROLLER_RA8875)
#define LV_HOR_RES_MAX 480
#define LV_VER_RES_MAX 320
#else
#define LV_HOR_RES_MAX 480
#define LV_VER_RES_MAX 272
#endif

#include "lv_conf.h"

// The shared colorlcd lv_conf.h sizes the static LVGL memory pool from the
// SDRAM_* macros: openx1 (8MB PSRAM, no SDRAM_16M/32M) gets LV_MEM=2, i.e. a
// 2MB static pool in PSRAM. That leaves only ~2.2MB of PSRAM as heap, which
// is too little for Lua Bitmap.open() of larger images (peak decode is high).
// Shrink the pool to 768KB to free more PSRAM for the heap. Tune this up
// if the UI ever runs low on LVGL memory.
#if !defined(SIMU)
#  undef LV_MEM_SIZE
#  define LV_MEM_SIZE (768U * 1024U)

// openx1 prioritizes large image loading over UI caching/perf.
// Keep LVGL's temporary buffers small and disable decoded image cache so
// memory is released promptly instead of being retained across draws.
#  undef LV_MEM_BUF_MAX_NUM
#  define LV_MEM_BUF_MAX_NUM 8

#  undef LV_LAYER_SIMPLE_BUF_SIZE
#  define LV_LAYER_SIMPLE_BUF_SIZE (8U * 1024U)

#  undef LV_LAYER_SIMPLE_FALLBACK_BUF_SIZE
#  define LV_LAYER_SIMPLE_FALLBACK_BUF_SIZE (1024U)

#  undef LV_IMG_CACHE_DEF_SIZE
#  define LV_IMG_CACHE_DEF_SIZE 0
#endif

#endif /*LV_CONF_EDGETX_H*/

#endif /*End of "Content enable"*/
