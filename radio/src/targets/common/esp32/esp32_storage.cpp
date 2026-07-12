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
