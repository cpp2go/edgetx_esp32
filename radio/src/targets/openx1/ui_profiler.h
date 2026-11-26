/*
 * Copyright (C) EdgeTX
 *
 * Based on code named
 *   opentx - https://github.com/opentx/opentx
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 */

#pragma once

#if defined(UI_FRAME_PROFILER)

// On-screen UI timing overlay, enabled with -DUI_FRAME_PROFILER (see
// esp32_build/CMakeLists.txt). Shows the frame rate plus a per-stage time
// breakdown so a slow screen can be attributed to LVGL rendering, the I80
// flush, the widget (Lua) refresh or the background script task.
//
// Kept out of hal.h: LVGL pulls hal.h in through lv_conf.h inside an
// extern "C" block, which would give these C++ helpers C linkage.
enum UiProfilerStage {
  UI_PROF_LVGL = 0,   // lv_timer_handler(): render + flush
  UI_PROF_UI,         // MainWindow::run()
  UI_PROF_LUA,        // luaTask()
  UI_PROF_FLUSH,      // the I80 flush inside UI_PROF_LVGL
  UI_PROF_LUA_DRAW,   // Lua widget refresh() inside UI_PROF_LVGL
  UI_PROF_STAGE_COUNT,
};

// Drawing primitives, timed separately so a slow screen can be attributed to
// text, solid fills, images (alpha blits), shapes or lines.
enum UiProfilerDraw {
  UI_DRAW_TEXT = 0,
  UI_DRAW_RECT,
  UI_DRAW_IMAGE,
  UI_DRAW_SHAPE,
  UI_DRAW_LINE,
  UI_DRAW_COUNT,
};

void uiProfilerEnter(unsigned int stage);
void uiProfilerExit(unsigned int stage);
void uiProfilerCount(unsigned int stage);
void uiProfilerLuaInstructions(unsigned int percent);
void uiProfilerFrameEnd();

// Standalone Lua scripts: how often run() actually gets called per second (the
// overlay shows it, so a slow script can be told apart from a slow module).
void uiProfilerLuaIteration();

void uiProfilerDrawEnter(unsigned int kind);
void uiProfilerDrawExit(unsigned int kind);

// Times the enclosing function, including early returns.
struct UiProfilerDrawScope {
  unsigned int kind;
  explicit UiProfilerDrawScope(unsigned int k) : kind(k)
  {
    uiProfilerDrawEnter(kind);
  }
  ~UiProfilerDrawScope() { uiProfilerDrawExit(kind); }
};

#define UI_PROF_DRAW_SCOPE(kind) UiProfilerDrawScope _uiProfDrawScope(kind)

#endif  // UI_FRAME_PROFILER
