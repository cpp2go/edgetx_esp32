/*
 * Compatibility shim: radio/src/thirdparty/ is in the include path before
 * both thirdparty/FatFs/ (STM32) and esp-idf fatfs/src/ (ESP32), so this
 * file is found first by any #include "ff.h".
 *
 * #include_next forwards to the *next* ff.h in the search path:
 *   - ESP32  builds: esp-idf/components/fatfs/src/ff.h  (defines FF_DIR, not DIR)
 *   - STM32  builds: thirdparty/FatFs/ff.h              (defines DIR already)
 *
 * Detection: ESP-IDF's ffconf.h (included by ff.h) defines _FFCONF_DEFINED;
 * the thirdparty FatFs ffconf.h does not.  Use that to decide whether to
 * add the DIR compat typedef.
 */

#pragma once

#if defined(ESP_PLATFORM) || defined(IDF_VER)
/* Prefer the ESP-IDF FatFS header explicitly for ESP32 builds. The shim itself
 * is named ff.h, so a plain __has_include(<ff.h>) probe can resolve to this
 * file instead of the real FatFS header. */
#  if defined(__has_include)
#    if __has_include("C:/esp/v6.0.1/esp-idf/components/fatfs/src/ff.h")
#      include "C:/esp/v6.0.1/esp-idf/components/fatfs/src/ff.h"
#    elif __has_include_next("ff.h")
#      include_next "ff.h"
#    elif __has_include("FatFs/ff.h")
#      include "FatFs/ff.h"
#    endif
#  else
#    include "C:/esp/v6.0.1/esp-idf/components/fatfs/src/ff.h"
#  endif
#elif defined(__has_include_next)
#  include_next "ff.h"
#elif defined(__has_include)
#  if __has_include("FatFs/ff.h")
#    include "FatFs/ff.h"
#  endif
#else
#  include "FatFs/ff.h"
#endif

/* Add DIR alias only when ESP-IDF's FatFS was actually included */
#if defined(_FFCONF_DEFINED)
typedef FF_DIR DIR;

/* ESP-IDF FatFS only defines FF_MAX_LFN when LFN is enabled.
 * Provide a fallback so code using FF_MAX_LFN compiles with LFN disabled. */
#ifndef FF_MAX_LFN
#  ifdef CONFIG_FATFS_MAX_LFN
#    define FF_MAX_LFN CONFIG_FATFS_MAX_LFN
#  else
#    define FF_MAX_LFN 255
#  endif
#endif

/* ESP-IDF hard-codes FF_FS_RPATH = 0, so f_chdir / f_getcwd are not
 * declared in its ff.h.  Declare them here; implementations are in
 * targets/common/esp32/esp32_storage.cpp. */
#ifdef __cplusplus
extern "C" {
#endif
FRESULT f_chdir(const TCHAR* path);
FRESULT f_getcwd(TCHAR* buff, UINT len);
#ifdef __cplusplus
}
#endif

#endif
