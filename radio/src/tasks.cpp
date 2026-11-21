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

#include "debug.h"
#include "edgetx.h"
#if defined(ESP_PLATFORM)
#include "esp_log.h"
#endif
#include "os/sleep.h"
#include "os/task.h"
#include "os/time.h"
#include "os/timer.h"
#include "timers_driver.h"
#include "hal/abnormal_reboot.h"
#include "hal/watchdog_driver.h"
#include "pdm_wav_recorder.h"

#include "tasks.h"
#include "tasks/mixer_task.h"


#if defined(COLORLCD)
#include "startup_shutdown.h"
#endif

#include "targets/openx1/diag_stats.h"

#if defined(OPENX1_DIAG_STATS)
#include "esp_timer.h"
#include "esp_log.h"

diag_stat_t diagStats[DIAG_ID_COUNT] = {};
uint32_t diagLcdPixels;
uint32_t diagLcdMaxPixels;
int32_t diagLcdMaxW;
int32_t diagLcdMaxH;

static const char* const diagNames[DIAG_ID_COUNT] = {
    "key10ms", "perMain", "audio", "lcdflush"};

void diagEnter(DiagId id)
{
  int64_t now = esp_timer_get_time();
  diagStats[id].enterUs = now;
  if (diagStats[id].lastUs) {
    uint32_t gap = (uint32_t)((now - diagStats[id].lastUs) / 1000);
    if (gap > diagStats[id].maxGapMs) diagStats[id].maxGapMs = gap;
  }
  diagStats[id].lastUs = now;
  diagStats[id].count++;
}

void diagExit(DiagId id)
{
  uint32_t dur = (uint32_t)((esp_timer_get_time() - diagStats[id].enterUs) / 1000);
  if (dur > diagStats[id].maxDurMs) diagStats[id].maxDurMs = dur;
}

void diagReport()
{
  static int64_t reportAt = 0;
  int64_t now = esp_timer_get_time();
  if (!reportAt) {
    reportAt = now;
    return;
  }
  if (now - reportAt < 1000000) return;
  reportAt = now;

  char buf[192];
  int n = 0;
  for (int i = 0; i < DIAG_ID_COUNT && n < (int)sizeof(buf) - 48; i++) {
    n += snprintf(buf + n, sizeof(buf) - n, " %s:%u/%ums/%ums",
                  diagNames[i], (unsigned)diagStats[i].count,
                  (unsigned)diagStats[i].maxGapMs,
                  (unsigned)diagStats[i].maxDurMs);
    diagStats[i].maxGapMs = diagStats[i].maxDurMs = 0;
    diagStats[i].count = 0;
  }
  // Flushed pixels: total over the second and the largest single update, to
  // tell a full-screen redraw from a small partial one.
  n += snprintf(buf + n, sizeof(buf) - n, " px=%uK max=%ux%u",
                (unsigned)(diagLcdPixels / 1000), (int)diagLcdMaxW,
                (int)diagLcdMaxH);
  diagLcdPixels = 0;
  diagLcdMaxPixels = 0;
  diagLcdMaxW = diagLcdMaxH = 0;

  ESP_LOGI("DIAG", "n/gap/dur%s", buf);
}
#endif

task_handle_t menusTaskId;
TASK_DEFINE_STACK(menusStack, MENUS_STACK_SIZE);

#if defined(AUDIO)
task_handle_t audioTaskId;
TASK_DEFINE_STACK(audioStack, AUDIO_STACK_SIZE);
#endif

mutex_handle_t audioMutex;

#if defined(ESP_PLATFORM) && defined(COLORLCD)
// LVGL default refresh is 30ms (sdkconfig); 50ms capped the UI at ~20 FPS.
#define MENU_TASK_PERIOD (33)
#else
#define MENU_TASK_PERIOD (50)  // 50ms
#endif

#if defined(COLORLCD) && defined(CLI)
bool perMainEnabled = true;
#endif

static void menusTask()
{
#if defined(COLORLCD)
  LvglWrapper::instance();
#endif

  edgeTxInit();

  mixerTaskInit();

#if defined(PWR_BUTTON_PRESS)
  static uint32_t pwr_press_logged = 0;
  while (task_running()) {
    uint32_t pwr_check = pwrCheck();
    if (pwr_check == e_power_off) {
      TRACE("PWR: boardOff triggered (pwrCheck=e_power_off)");
      break;
    } else if (pwr_check == e_power_press) {
      if (!pwr_press_logged) {
        TRACE("PWR: pwrPressed=true, shutdown countdown started");
        pwr_press_logged = 1;
      }
      sleep_ms(MENU_TASK_PERIOD);
      continue;
    }
    pwr_press_logged = 0;
#else
  while (pwrCheck() != e_power_off) {
#endif
    time_point_t next_tick = time_point_now();
    DEBUG_TIMER_START(debugTimerPerMain);
    diagEnter(DIAG_PERMAIN);
#if defined(COLORLCD) && defined(CLI)
    if (perMainEnabled) {
      perMain();
    }
#else
    perMain();
#endif
    diagExit(DIAG_PERMAIN);
    DEBUG_TIMER_STOP(debugTimerPerMain);

    sleep_until(&next_tick, MENU_TASK_PERIOD);
    resetForcePowerOffRequest();
  }

#if defined(PCBX9E)
  toplcdOff();
#endif

#if !defined(PWR_BUTTON_MANAGED)
  drawSleepBitmap();
#endif
  edgeTxClose();
  boardOff();
}

static void audioTask()
{
  while (!audioQueue.started()) {
    sleep_ms(1);
  }

#if defined(PCBX12S) || defined(RADIO_TX16S) || defined(RADIO_F16) || defined(RADIO_V16)
  // The audio amp needs ~2s to start
  sleep_ms(1000); // 1s
#endif

  time_point_t next_tick = time_point_now();
  while (task_running()) {
    DEBUG_TIMER_SAMPLE(debugTimerAudioIterval);
    DEBUG_TIMER_START(debugTimerAudioDuration);
    diagEnter(DIAG_AUDIO);
    audioQueue.wakeup();
    diagExit(DIAG_AUDIO);
    diagReport();
#if defined(PDM_CLOCK)
    // Drive microphone recording (if any) at the audio-task cadence.
    // Much shorter than the UI tick interval, so the PDM ring buffer
    // never fills enough to trigger skip-ahead / sample drops.
    PdmWavRecorder::audioTick();
#endif
    DEBUG_TIMER_STOP(debugTimerAudioDuration);
    sleep_until(&next_tick, 4);
  }
}

static timer_handle_t _timer10ms = TIMER_INITIALIZER;

static void _timer_10ms_cb(timer_handle_t* h)
{
  per10ms();
}

static void timer10msStart()
{
  if (!timer_is_created(&_timer10ms)) {
    timer_create(&_timer10ms, _timer_10ms_cb, "10ms", 10, true);
  }

  timer_start(&_timer10ms);
}

void tasksStart()
{
  mutex_create(&audioMutex);

#if defined(CLI) && !defined(SIMU)
  cliStart();
#endif

#if !defined(ESP_PLATFORM)
  timer10msStart();
#endif

  task_create(&menusTaskId, menusTask, "menus", menusStack, MENUS_STACK_SIZE,
              MENUS_TASK_PRIO,
#if defined(MENU_TASK_CORE)
              MENU_TASK_CORE
#else
              TASK_NO_AFFINITY
#endif
  );

#if defined(AUDIO)
  task_create(&audioTaskId, audioTask, "audio", audioStack, AUDIO_STACK_SIZE,
              AUDIO_TASK_PRIO,
#if defined(AUDIO_TASK_CORE)
              AUDIO_TASK_CORE
#else
              TASK_NO_AFFINITY
#endif
  );
#endif

  RTOS_START();
}
