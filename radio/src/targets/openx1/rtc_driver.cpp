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
#include "esp_rtc_time.h"
#include "nvs_flash.h"

#include <string.h>
#include <sys/time.h>

/*
 * OpenX1 RTC driver - no external DS3231 needed.
 *
 * The ESP32-P4 module carries an internal (battery/VBAT backed) RTC counter
 * that keeps counting while the module is powered down.  ESP-IDF exposes it
 * as esp_rtc_get_time_us().  We keep a reference sample (rtc_us) together
 * with the matching Unix epoch and persist both in NVS, so the wall-clock
 * time can be recovered on every boot:
 *
 *   epoch_now = stored_epoch + (esp_rtc_get_time_us() - stored_rtc_us) / 1e6
 *
 * The first time (or after a full power loss without VBAT) the counter has no
 * meaningful base, so the radio just reports the last stored epoch until the
 * user / USB / Wi-Fi sets a new time via rtcSetTime().
 */

#define RTC_SYNC_KEY       "rtc_time"
#define RTC_SYNC_NAMESPACE "nvs"

typedef struct {
  int64_t rtc_us;    /* esp_rtc_get_time_us() at the moment of sync   */
  int64_t epoch;     /* matching Unix epoch (seconds, UTC)            */
} rtc_sync_t;

static void rtc_sync_load(rtc_sync_t *sync)
{
  nvs_handle_t h;
  if (nvs_open(RTC_SYNC_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
    memset(sync, 0, sizeof(*sync));
    return;
  }
  size_t len = sizeof(*sync);
  if (nvs_get_blob(h, RTC_SYNC_KEY, sync, &len) != ESP_OK) {
    memset(sync, 0, sizeof(*sync));
  }
  nvs_close(h);
}

static void rtc_sync_save(const rtc_sync_t *sync)
{
  nvs_handle_t h;
  if (nvs_open(RTC_SYNC_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
    return;
  }
  nvs_set_blob(h, RTC_SYNC_KEY, sync, sizeof(*sync));
  nvs_commit(h);
  nvs_close(h);
}

/* Sync the ESP-IDF system clock (used by time()/FTP/Lua) with g_rtcTime */
static void rtc_sync_system_clock(gtime_t epoch)
{
  struct timeval tv;
  tv.tv_sec = (time_t)epoch;
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);
}

void rtcSetTime(const struct gtm * t)
{
  gtime_t epoch = gmktime((struct gtm *)t);
  TRACE("rtcSetTime %d/%d/%d %d:%d:%d -> %ld",
        t->tm_year + TM_YEAR_BASE, t->tm_mon + 1, t->tm_mday,
        t->tm_hour, t->tm_min, t->tm_sec, (long)epoch);

  rtc_sync_t sync;
  sync.rtc_us = (int64_t)esp_rtc_get_time_us();
  sync.epoch = epoch;
  rtc_sync_save(&sync);

  rtc_sync_system_clock(epoch);
}

void rtcGetTime(struct gtm * t)
{
  gettime(t);
}

void rtcInit()
{
  g_rtcTime = 0;

  rtc_sync_t sync;
  rtc_sync_load(&sync);
  if (sync.rtc_us > 0 && sync.epoch > 0) {
    // Reconstruct the current epoch from the free-running internal RTC.
    int64_t delta_us = (int64_t)esp_rtc_get_time_us() - sync.rtc_us;
    if (delta_us < 0) {
      // RTC counter was reset (full power loss w/o VBAT): keep last epoch.
      delta_us = 0;
    }
    g_rtcTime = sync.epoch + delta_us / 1000000LL;
  }

  rtc_sync_system_clock(g_rtcTime > 0 ? g_rtcTime : 0);
  TRACE("rtcInit: epoch=%ld (internal P4 RTC)", (long)g_rtcTime);
}
