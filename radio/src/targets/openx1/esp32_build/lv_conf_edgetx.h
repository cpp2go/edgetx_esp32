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
// is too little for Lua Bitmap.open() of larger images (peak ~6xWxH during
// decode). Shrink the pool to 1MB to free PSRAM for the heap. Tune this up
// if the UI ever runs low on LVGL memory.
#if !defined(SIMU)
#  undef LV_MEM_SIZE
#  define LV_MEM_SIZE (1 * 1024U * 1024U)
#endif

#endif /*LV_CONF_EDGETX_H*/

#endif /*End of "Content enable"*/
