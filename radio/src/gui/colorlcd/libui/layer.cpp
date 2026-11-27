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

#include "layer.h"

std::list<Layer> Layer::stack;

Layer::Layer(Window* w, lv_group_t* g, lv_group_t* pg)
    : window(w), group(g), prevGroup(pg),
      prevEditing(pg != nullptr && lv_group_get_editing(pg)) {}
Layer::~Layer() { lv_group_del(group); }

void _assign_lv_group(lv_group_t* g)
{
  lv_group_set_default(g);

  // associate it with all input devices
  lv_indev_t* indev = lv_indev_get_next(NULL);
  while (indev) {
    lv_indev_set_group(indev, g);
    indev = lv_indev_get_next(indev);
  }
}

// An encoder only drives whatever object the active group has focused. When a
// layer is popped (e.g. after a Lua script exits) the group that comes back can
// be left without focus - the page still works with the keys (they fall back to
// Layer::back() dispatch) but the encoder does nothing until the layer is
// entered again. Fix whatever group each input device is actually bound to.
//
// ESP32 only: the other targets have run this code for years without it, so
// their behaviour is left untouched.
#if defined(ESP_PLATFORM)
static void _fix_group_focus(lv_group_t* g)
{
  if (g && !lv_group_get_focused(g) && lv_group_get_obj_count(g)) {
    lv_group_focus_next(g);
  }
}

static void _ensure_group_focus(lv_group_t* g)
{
  _fix_group_focus(g);

  lv_indev_t* indev = lv_indev_get_next(NULL);
  while (indev) {
    if (indev->group && indev->group != g) _fix_group_focus(indev->group);
    indev = lv_indev_get_next(indev);
  }
}

// Returning from a sub-layer restores the edit mode the group had before the
// layer was pushed, and re-focuses it if the focus got lost. A layer can leave
// edit mode on (a standalone Lua script sets it on its own group), which must
// not leak into the page below: with editing on, the encoder edits the focused
// value instead of moving the focus, which on a plain list page looks exactly
// like a dead encoder. Clearing it unconditionally is just as wrong though: a
// page whose encoder navigation *needs* edit mode (the SD manager's file
// browser, see TableField::setAutoEdit) would then look dead instead.
static void _restore_group(lv_group_t* g, bool editing)
{
  if (!g) return;

  lv_group_set_editing(g, editing);
  _ensure_group_focus(g);
}
#else
static inline void _restore_group(lv_group_t*, bool) {}
#endif

void Layer::push(Window* w)
{
  // save prev group
  auto pg = lv_group_get_default();

  // create a new group
  auto g = lv_group_create();
  _assign_lv_group(g);
  
  // and store
  stack.emplace_back(w, g, pg);
}

void Layer::pop(Window* w)
{
  if (stack.empty()) return;

  if (back() == w) {
    lv_group_t* prevGroup = stack.back().prevGroup;
    bool prevEditing = stack.back().prevEditing;
    if (prevGroup) {
      _assign_lv_group(prevGroup);
      _restore_group(prevGroup, prevEditing);
    } else if (!stack.empty()) {
      _assign_lv_group(stack.back().group);
      _restore_group(stack.back().group, false);
    } else {
      lv_group_set_default(NULL);
    }
    stack.pop_back();
  } else {
    for (auto layer = stack.crbegin(); layer != stack.crend(); layer++) {
      if (layer->window == w) {
        stack.erase(std::next(layer).base());
        return;
      }
    }
    return;
  }
}

Window* Layer::back()
{
  if (stack.empty()) return nullptr;
  return stack.back().window;
}

Window* Layer::getFirstOpaque()
{
  for (auto layer = stack.crbegin(); layer != stack.crend(); layer++) {
    auto w = layer->window;
    if (w && w->hasWindowFlag(OPAQUE)) return w;
  }

  return nullptr;
}

Window* Layer::walk(std::function<bool(Window* w)> check)
{
  for (auto layer = stack.crbegin(); layer != stack.crend(); layer++) {
    if (layer->window && check(layer->window))
      return layer->window;
  }

  return nullptr;
}

Window* Layer::getPageGroup()
{
  Window* w = Layer::walk([=](Window *w) mutable -> bool {
    return w->isPageGroup();
  });
  return w;
}
