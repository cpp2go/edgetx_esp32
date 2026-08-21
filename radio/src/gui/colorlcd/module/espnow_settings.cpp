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

#include "espnow_settings.h"

#include "button.h"
#include "choice.h"
#include "edgetx.h"
#include "numberedit.h"
#include "static.h"

#include "hal/module_driver.h"
#include "esprc.h"

#define SET_DIRTY() storageDirty(EE_MODEL)

/* Map ESP-NOW RSSI (dBm, typically -30..-100) to a 0..100 link quality */
static uint8_t espnow_rssi_to_pct(int8_t rssi_dbm)
{
  int32_t pct = 100 - (((int32_t)(-rssi_dbm) - 30) * 100) / 70;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (uint8_t)pct;
}

static void onBind()
{
  init_bind_espnow();
}

EspNowSettings::EspNowSettings(Window* parent, const FlexGridLayout& g,
                               uint8_t moduleIdx) :
    Window(parent, rect_t{}), moduleIdx(moduleIdx)
{
  md = &g_model.moduleData[moduleIdx];

  setFlexLayout(LV_FLEX_FLOW_COLUMN, PAD_TINY);

  FlexGridLayout grid(g);
  FormLine* line;

  // Channel
  line = newLine(grid);
  new StaticText(line, rect_t{}, "Channel");
  new NumberEdit(line, rect_t{}, 1, 13,
                 GET_DEFAULT(md->espnow.ch),
                 [=](int32_t newValue) {
                   md->espnow.ch = newValue;
                   SET_DIRTY();
                 });

  // RX MAC address display
  line = newLine(grid);
  new StaticText(line, rect_t{}, "RX MAC");

  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           md->espnow.rx_mac_addr[0], md->espnow.rx_mac_addr[1],
           md->espnow.rx_mac_addr[2], md->espnow.rx_mac_addr[3],
           md->espnow.rx_mac_addr[4], md->espnow.rx_mac_addr[5]);
  new StaticText(line, rect_t{}, macStr);

  // Link status
  line = newLine(grid);
  new StaticText(line, rect_t{}, "Status");
  statusText = new StaticText(line, rect_t{}, "Disconnected");

  // RSSI
  line = newLine(grid);
  new StaticText(line, rect_t{}, "RSSI");
  rssiText = new StaticText(line, rect_t{}, "N/A");

  // Link quality
  line = newLine(grid);
  new StaticText(line, rect_t{}, "Link");
  linkText = new StaticText(line, rect_t{}, "0%");

  // Packets sent / acknowledged
  line = newLine(grid);
  new StaticText(line, rect_t{}, "Packets");
  pktText = new StaticText(line, rect_t{}, "0/0");

  // Bind button
  line = newLine(grid);
  new StaticText(line, rect_t{}, "");
  bindBtn = new TextButton(line, rect_t{}, STR_BIND, [=]() -> uint8_t {
    onBind();
    return 0;
  });
}

void EspNowSettings::update()
{
  // Update link status
  if (espnowLinkState) {
    statusText->setText("Connected");
  } else {
    statusText->setText("Disconnected");
  }

  // Update RSSI
  char buf[16];
  snprintf(buf, sizeof(buf), "%d dBm", espnowRssi);
  rssiText->setText(buf);

  // Update link quality
  snprintf(buf, sizeof(buf), "%d%%", espnow_rssi_to_pct(espnowRssi));
  linkText->setText(buf);

  // Update packet stats
  snprintf(buf, sizeof(buf), "%lu/%lu", packSent, packAckn);
  pktText->setText(buf);
}
