/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

// ESP32-specific storage and FatFS relative-path support.
//
// hal/storage.cpp is excluded from ESP32 builds because it registers
// drivers with the thirdparty FatFs.  On ESP32, ESP-IDF's FatFS uses
// ff_diskio_register() instead of the custom fatfsRegisterDriver().
//
// f_getcwd / f_chdir are compiled into ff.c only when FF_FS_RPATH >= 2.
// ESP-IDF hard-codes FF_FS_RPATH = 0 with no Kconfig knob, so we
// provide our own implementations that track CWD in a static string.
// f_opendir() is not affected as long as callers pass absolute paths.

#include <string.h>

#include "ff.h"
#include "diskio_impl.h"
#include "diskio_spi.h"
#include "hal/storage.h"

// ─── Storage API ──────────────────────────────────────────────────────────────

static DSTATUS esp32_disk_init(unsigned char pdrv)
{
  return sdcard_spi_driver.initialize(pdrv);
}
static DSTATUS esp32_disk_status(unsigned char pdrv)
{
  return sdcard_spi_driver.status(pdrv);
}
static DRESULT esp32_disk_read(unsigned char pdrv, unsigned char* buff,
                               uint32_t sector, unsigned count)
{
  return sdcard_spi_driver.read(pdrv, buff, sector, count);
}
static DRESULT esp32_disk_write(unsigned char pdrv, const unsigned char* buff,
                                uint32_t sector, unsigned count)
{
  return sdcard_spi_driver.write(pdrv, buff, sector, count);
}
static DRESULT esp32_disk_ioctl(unsigned char pdrv, unsigned char cmd,
                                void* buff)
{
  return sdcard_spi_driver.ioctl(pdrv, cmd, buff);
}

static const ff_diskio_impl_t s_sdcard_impl = {
    .init   = esp32_disk_init,
    .status = esp32_disk_status,
    .read   = esp32_disk_read,
    .write  = esp32_disk_write,
    .ioctl  = esp32_disk_ioctl,
};

void storageInit()
{
  ff_diskio_register(0, &s_sdcard_impl);
}

void storageDeInit()
{
  ff_diskio_register(0, nullptr);
}

void storagePreMountHook() {}

bool storageIsPresent()
{
  return (sdcard_spi_driver.status(0) & STA_NODISK) == 0;
}

const diskio_driver_t* storageGetDefaultDriver()
{
  return &sdcard_spi_driver;
}

// ─── f_getcwd / f_chdir (FF_FS_RPATH = 0 on ESP-IDF) ─────────────────────────
//
// These are declared in ff.h only when FF_FS_RPATH >= 1/2.  Since
// ESP-IDF's ffconf.h hard-codes FF_FS_RPATH = 0, the declarations are
// absent and the functions are not compiled into ff.c.  We provide them
// here.  Callers that use f_opendir() must pass the full absolute path
// returned by f_getcwd() rather than ".".

static char s_cwd[512] = "/";

extern "C" FRESULT f_getcwd(TCHAR* buff, UINT len)
{
  if (!buff || len == 0) return FR_INVALID_PARAMETER;
  strncpy(buff, s_cwd, len - 1);
  buff[len - 1] = '\0';
  return FR_OK;
}

extern "C" FRESULT f_chdir(const TCHAR* path)
{
  if (!path || !path[0]) return FR_INVALID_NAME;

  if (path[0] == '/') {
    // Absolute path — resolve into s_cwd, handling ".." and "." components
    char tmp[512];
    tmp[0] = '/';
    size_t len = 1;
    tmp[1] = '\0';

    const char* p = path + 1;
    while (*p) {
      const char* seg = p;
      while (*p && *p != '/') p++;
      size_t seglen = p - seg;

      if (seglen == 2 && seg[0] == '.' && seg[1] == '.') {
        // go up one level
        char* slash = strrchr(tmp, '/');
        if (slash && slash != tmp) {
          *slash = '\0';
          len = (size_t)(slash - tmp);
        } else {
          tmp[0] = '/'; tmp[1] = '\0'; len = 1;
        }
      } else if (seglen == 1 && seg[0] == '.') {
        // stay — skip
      } else if (seglen > 0 && len + 1 + seglen < sizeof(tmp)) {
        if (len > 1) tmp[len++] = '/';
        memcpy(tmp + len, seg, seglen);
        len += seglen;
        tmp[len] = '\0';
      }

      if (*p == '/') p++;
    }
    strncpy(s_cwd, tmp, sizeof(s_cwd) - 1);
    s_cwd[sizeof(s_cwd) - 1] = '\0';
  } else if (path[0] == '.' && path[1] == '.' &&
             (path[2] == '/' || path[2] == '\0')) {
    // ".." – go up one level
    char* slash = strrchr(s_cwd, '/');
    if (slash && slash != s_cwd) {
      *slash = '\0';
    } else {
      s_cwd[0] = '/';
      s_cwd[1] = '\0';
    }
    if (path[2] == '/') {
      return f_chdir(path + 3);
    }
  } else {
    // relative segment – append to CWD
    size_t cwdlen = strlen(s_cwd);
    if (cwdlen > 1) {
      strncat(s_cwd, "/", sizeof(s_cwd) - cwdlen - 1);
      cwdlen++;
    }
    strncat(s_cwd, path, sizeof(s_cwd) - cwdlen - 1);
  }
  return FR_OK;
}

// ─── Relative path support for FatFS (FF_FS_RPATH = 0) ──────────────────────
//
// ESP-IDF hard-codes FF_FS_RPATH = 0, so FatFS f_open()/f_stat()/... cannot
// resolve relative paths against a working directory. Lua scripts call
// chdir() (our f_chdir stub, which only tracks s_cwd) and then use relative
// file/image paths — those would fail on ESP32.  To make them work the same
// way as on RPATH-enabled radios, the linker wraps the FatFS entry points
// used by the Lua file/image APIs (see esp32_build/CMakeLists.txt --wrap)
// so that relative paths are resolved against s_cwd before reaching FatFS.

#define ESP32_RESOLVE_BUF   (sizeof(s_cwd) + FF_MAX_LFN + 2)

// Normalize an absolute path in place: collapse duplicate slashes and
// resolve "." / ".." segments.  FatFS has FF_FS_RPATH=0 and cannot do
// this itself, so "./test.png" or "a/../b" must be cleaned up here.
static void esp32_normalize_path(char* path)
{
  char tmp[ESP32_RESOLVE_BUF];
  tmp[0] = '/';
  tmp[1] = '\0';
  size_t len = 1;

  const char* p = path;
  while (*p) {
    while (*p == '/') p++;   // skip slashes
    if (!*p) break;
    const char* seg = p;
    while (*p && *p != '/') p++;
    size_t seglen = (size_t)(p - seg);

    if (seglen == 2 && seg[0] == '.' && seg[1] == '.') {
      // go up one level
      char* slash = strrchr(tmp, '/');
      if (slash && slash != tmp) {
        *slash = '\0';
        len = (size_t)(slash - tmp);
      } else {
        tmp[0] = '/'; tmp[1] = '\0'; len = 1;
      }
    } else if (seglen == 1 && seg[0] == '.') {
      // stay — skip
    } else if (seglen > 0 && len + 1 + seglen < sizeof(tmp)) {
      if (len > 1) tmp[len++] = '/';
      memcpy(tmp + len, seg, seglen);
      len += seglen;
      tmp[len] = '\0';
    }
  }
  strcpy(path, tmp);
}

// Resolve a possibly-relative path against s_cwd. Absolute paths
// ("/..." or "x:/...") are returned unchanged (after normalization).
// Returns the original pointer when the path needs no resolution,
// otherwise fills `buf`.
static const TCHAR* esp32_resolve_path(const TCHAR* path, char* buf,
                                       size_t bufsize)
{
  if (!path || !path[0] || strchr(path, ':') || bufsize < 2)
    return path;

  if (path[0] == '/') {
    // already absolute — normalize "." / ".." segments
    strncpy(buf, path, bufsize - 1);
    buf[bufsize - 1] = '\0';
    esp32_normalize_path(buf);
    return buf;
  }

  // relative: prepend CWD
  if (f_getcwd(buf, bufsize - 1) != FR_OK || !buf[0])
    return path;

  size_t len = strlen(buf);
  if (len > 1 && buf[len - 1] != '/')
    buf[len++] = '/';
  if (len + 1 >= bufsize)
    return path;
  strncat(buf, path, bufsize - len - 1);
  esp32_normalize_path(buf);
  return buf;
}

extern "C" FRESULT __real_f_open(FIL* fp, const TCHAR* path, BYTE mode);
extern "C" FRESULT __wrap_f_open(FIL* fp, const TCHAR* path, BYTE mode)
{
  char buf[ESP32_RESOLVE_BUF];
  return __real_f_open(fp, esp32_resolve_path(path, buf, sizeof(buf)), mode);
}

extern "C" FRESULT __real_f_stat(const TCHAR* path, FILINFO* fno);
extern "C" FRESULT __wrap_f_stat(const TCHAR* path, FILINFO* fno)
{
  char buf[ESP32_RESOLVE_BUF];
  return __real_f_stat(esp32_resolve_path(path, buf, sizeof(buf)), fno);
}

extern "C" FRESULT __real_f_opendir(FF_DIR* dp, const TCHAR* path);
extern "C" FRESULT __wrap_f_opendir(FF_DIR* dp, const TCHAR* path)
{
  char buf[ESP32_RESOLVE_BUF];
  return __real_f_opendir(dp, esp32_resolve_path(path, buf, sizeof(buf)));
}

extern "C" FRESULT __real_f_unlink(const TCHAR* path);
extern "C" FRESULT __wrap_f_unlink(const TCHAR* path)
{
  char buf[ESP32_RESOLVE_BUF];
  return __real_f_unlink(esp32_resolve_path(path, buf, sizeof(buf)));
}

extern "C" FRESULT __real_f_mkdir(const TCHAR* path);
extern "C" FRESULT __wrap_f_mkdir(const TCHAR* path)
{
  char buf[ESP32_RESOLVE_BUF];
  return __real_f_mkdir(esp32_resolve_path(path, buf, sizeof(buf)));
}

extern "C" FRESULT __real_f_rename(const TCHAR* path_old, const TCHAR* path_new);
extern "C" FRESULT __wrap_f_rename(const TCHAR* path_old, const TCHAR* path_new)
{
  char oldBuf[ESP32_RESOLVE_BUF];
  char newBuf[ESP32_RESOLVE_BUF];
  return __real_f_rename(esp32_resolve_path(path_old, oldBuf, sizeof(oldBuf)),
                         esp32_resolve_path(path_new, newBuf, sizeof(newBuf)));
}
