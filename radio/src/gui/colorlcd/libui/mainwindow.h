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

#pragma once

#include "layer.h"
#include "window.h"

class MainWindow: public Window
{
 protected:
  // singleton
  MainWindow();

 public:
  static MainWindow * instance();

#if defined(TESTS)
  static void create()
  {
    _instance = new MainWindow();
  }
#endif

#if defined(DEBUG_WINDOWS)
  std::string getName() const override
  {
    return "MainWindow";
  }
#endif

  void setActiveScreen() {
    lv_scr_load(lvobj);
  }

  void run(bool trash=true);

  void setBackgroundImage(const char* fileName);

  // Hides the window background (solid colour + background image) while a
  // fullscreen widget covers the whole screen: LVGL would otherwise repaint
  // them every frame only to have the widget draw over them again, which costs
  // a full-screen fill + image blend per frame on slow targets.
  void setBackgroundVisible(bool visible);

  void shutdown();

  void enableWidgetRefresh(bool state) { widgetRefreshEnable = state; }

 protected:
  std::string backgroundImageFileName;
  BitmapBuffer *backgroundBitmap = nullptr;
  lv_obj_t *backgroundCanvas = nullptr;
  bool backgroundVisible = true;
  bool widgetRefreshEnable = true;

  static MainWindow * _instance;
  static void emptyTrash();
};
