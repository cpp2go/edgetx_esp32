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

#include "simulib.h"

#include "hal/switch_driver.h"
#include "definitions.h"
#include "myeeprom.h"
#include "switches.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

struct hw_switch_def {
  const char*   name;
  SwitchHwType  type;
  SwitchConfig  defaultType;
#if defined(FUNCTION_SWITCHES)
  bool          isCustomSwitch;
  uint8_t       customSwitchIdx;
#endif
};

#include "simu_switches.inc"

int8_t switchesStates[MAX_SWITCHES];

// Where the Android host says the switches are. Weak with a default returning false, so
// the desktop simulator - which does not link android_host.cpp - builds and behaves
// exactly as before.
__attribute__((weak))
bool edgetxAndroidGetSwitchState(uint8_t idx, int8_t* state)
{
  (void)idx;
  (void)state;
  return false;
}

#if defined(RADIO_GX12)
void _poll_switches() {}
#endif

void simuSetSwitch(uint8_t swtch, int8_t state)
{
  assert(swtch < switchGetMaxAllSwitches());
  switchesStates[swtch] = state;
}

void boardInitSwitches() {
  memset(switchesStates, -1, sizeof(switchesStates));

  // This runs twice while booting (simuInit() and boardInit()), and the app has already
  // pushed the RC's real switch positions by then - so put the host's values back, or
  // every switch the app drives would show as "up" until it was moved again.
  for (uint8_t i = 0; i < MAX_SWITCHES; i++) {
    int8_t state;
    if (edgetxAndroidGetSwitchState(i, &state)) switchesStates[i] = state;
  }
}

SwitchHwPos boardSwitchGetPosition(uint8_t idx)
{
  if (switchesStates[idx] < 0)
    return SWITCH_HW_UP;
  else if (switchesStates[idx] == 0)
    return SWITCH_HW_MID;
  else
    return SWITCH_HW_DOWN;
}

const char* boardSwitchGetName(uint8_t idx)
{
  return _switch_defs[idx].name;
}

SwitchHwType boardSwitchGetType(uint8_t idx)
{
  return _switch_defs[idx].type;
}

uint8_t boardGetMaxSwitches() { return n_switches; }

SwitchConfig boardSwitchGetDefaultConfig(uint8_t idx) { return _switch_defs[idx].defaultType; }

#if defined(FUNCTION_SWITCHES)
bool boardIsCustomSwitch(uint8_t idx) { return (idx < n_switches) ? _switch_defs[idx].isCustomSwitch : false; }
uint8_t boardGetCustomSwitchIdx(uint8_t idx) { return _switch_defs[idx].customSwitchIdx; }
#endif

#if !defined(COLORLCD)
switch_display_pos_t switchGetDisplayPosition(uint8_t idx)
{
  if (idx >= DIM(_switch_display)) return {0, 0};

  return _switch_display[idx];
}
#endif
