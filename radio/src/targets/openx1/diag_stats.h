/*
 * Copyright (C) EdgeTX
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * Temporary (opt-in) runtime statistics for tracking down intermittent input
 * lag / audio dropouts: measures the longest gap and the longest duration of
 * the periodic tasks and logs a summary once per second.
 */

#pragma once

#include <stdint.h>

#if defined(OPENX1_DIAG_STATS)

enum DiagId {
  DIAG_PER10MS = 0,  // per10ms() call in the 5ms task (keys polling)
  DIAG_PERMAIN,      // perMain() (menu/LVGL frame)
  DIAG_AUDIO,        // audioQueue.wakeup()
  DIAG_LCD_FLUSH,    // one full-screen/area flush to the panel
  DIAG_ID_COUNT
};

struct diag_stat_t {
  int64_t enterUs;
  int64_t lastUs;
  uint32_t maxGapMs;
  uint32_t maxDurMs;
  uint32_t count;
};

extern diag_stat_t diagStats[DIAG_ID_COUNT];

// Area accounting for the LCD flush (how much of the screen each update pushes)
extern uint32_t diagLcdPixels;
extern uint32_t diagLcdMaxPixels;
extern int32_t diagLcdMaxW;
extern int32_t diagLcdMaxH;

void diagEnter(DiagId id);
void diagExit(DiagId id);
void diagReport();

static inline void diagLcdArea(int32_t w, int32_t h)
{
  if (w <= 0 || h <= 0) return;
  uint32_t px = (uint32_t)w * (uint32_t)h;
  diagLcdPixels += px;
  if (px > diagLcdMaxPixels) {
    diagLcdMaxPixels = px;
    diagLcdMaxW = w;
    diagLcdMaxH = h;
  }
}

#else

#define diagEnter(id)
#define diagExit(id)
#define diagReport()
#define diagLcdArea(w, h)

#endif
