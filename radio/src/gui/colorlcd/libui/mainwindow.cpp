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

#include "mainwindow.h"

#include "board.h"
#include "keyboard_base.h"
#include "layout.h"
#include "etx_lv_theme.h"
#include "sdcard.h"
#include "view_main.h"

// timers_driver.h
uint32_t timersGetMsTick();

MainWindow* MainWindow::_instance = nullptr;

MainWindow* MainWindow::instance()
{
  if (!_instance) _instance = new MainWindow();
  return _instance;
}

MainWindow::MainWindow() : Window(nullptr, {0, 0, LCD_W, LCD_H})
{
  setWindowFlag(OPAQUE);

  etx_solid_bg(lvobj);
}

void MainWindow::emptyTrash()
{
  for (auto window : trash) {
    delete window;
  }
  trash.clear();
}

void MainWindow::run(bool trash)
{
  auto start = timersGetMsTick();

  if (widgetRefreshEnable)
    ViewMain::refreshWidgets();

  auto opaque = Layer::getFirstOpaque();
  if (opaque) {
    opaque->checkEvents();
  }

  auto copy = children;
  for (auto child : copy) {
    if (!child->deleted() && child->isBubblePopup()) {
      child->checkEvents();
    }
  }

  if (trash) emptyTrash();

  auto delta = timersGetMsTick() - start;
  if (delta > 10) {
    TRACE_WINDOWS("MainWindow::run took %dms", delta);
  }
}

void MainWindow::shutdown()
{
  // Called when USB is connected in SD card mode

  // Delete main view screens
  LayoutFactory::deleteCustomScreens();

  // clear layer stack first
  for (Window* w = Layer::back(); w; w = Layer::back()) {
    w->deleteLater();
    Layer::pop(w);
  }

  children.clear();
  clear();
  emptyTrash();

  backgroundCanvas = nullptr;
  backgroundBitmap = nullptr;
}

void MainWindow::setBackgroundImage(const char* fileName)
{
  // ensure you delete old bitmap
  if (backgroundBitmap != nullptr) {
    backgroundBitmap->removeCanvas();
    delete backgroundBitmap;
    backgroundBitmap = nullptr;
    backgroundCanvas = nullptr;
  }

  if (fileName == nullptr) fileName = "";

  backgroundImageFileName = fileName;

  // Try to load bitmap. If this fails backgroundBitmap will be NULL and default
  // will be loaded in update() method
  backgroundBitmap =
      BitmapBuffer::loadBitmap(backgroundImageFileName.c_str(), BMP_RGB565);

  if (!backgroundBitmap) {
    backgroundBitmap = BitmapBuffer::loadBitmap(
        THEMES_PATH "/EdgeTX/background.png", BMP_RGB565);
  }

  if (backgroundBitmap) {
    backgroundCanvas = backgroundBitmap->addCanvas(this);
    if (!backgroundVisible)
      lv_obj_add_flag(backgroundCanvas, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_background(backgroundCanvas);
  }
}

void MainWindow::setBackgroundVisible(bool visible)
{
  if (backgroundVisible == visible) return;
  backgroundVisible = visible;

  if (backgroundCanvas) {
    if (visible)
      lv_obj_clear_flag(backgroundCanvas, LV_OBJ_FLAG_HIDDEN);
    else
      lv_obj_add_flag(backgroundCanvas, LV_OBJ_FLAG_HIDDEN);
  }

  // Not opaque => LVGL no longer stops here, and with the background image
  // hidden there is nothing left to draw underneath the fullscreen widget.
  lv_obj_set_style_bg_opa(lvobj, visible ? LV_OPA_COVER : LV_OPA_TRANSP, 0);

  invalidate();
}
