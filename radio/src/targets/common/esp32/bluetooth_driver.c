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

/*
 * ESP32-S3 native Bluetooth (BLE) driver for the generic EdgeTX Bluetooth
 * feature.
 *
 * The ESP32-S3 has no classic Bluetooth (BR/EDR/SPP); it only supports
 * Bluetooth LE.  This driver therefore implements the EdgeTX Bluetooth driver
 * interface on top of the ESP32-S3's built-in BLE (NimBLE), which is already
 * enabled in the openx1 sdkconfig (CONFIG_BT_ENABLED=y, CONFIG_BT_NIMBLE_ENABLED=y).
 *
 * Design:
 *   - The radio acts as a BLE *peripheral* exposing a Nordic UART Service
 *     (NUS) "serial bridge": a BLE central (phone app, another radio, ...)
 *     connects and exchanges the raw trainer / telemetry byte stream.
 *   - bluetoothInit()/bluetoothDisable() start/stop advertising.
 *   - bluetoothWrite() sends bytes to the central as GATT notifications
 *     (trainer/telemetry frames), unless the data is an "AT+..." command.
 *   - The "AT+..." commands emitted by the shared bluetooth.cpp state machine
 *     are intercepted and answered with the responses the state machine
 *     expects (OK+..., Peripheral:..., Connected:...), so the shared code
 *     needs no changes.  "AT+NAME..." also renames the advertised device.
 *   - bluetoothRead() returns bytes received from the central (RX ring buffer).
 *
 * NOTE: The radio currently only implements the peripheral (slave) role.
 *   "Master" trainer mode (radio acting as BLE central scanning/connecting)
 *   is not implemented yet.
 *
 * NOTE: This feature and the "BT PowerUp" module both use the same NimBLE
 *   host.  Only one may be active at a time.
 */

#include "bluetooth_driver.h"

#include <string.h>
#include <assert.h>
#include <stdio.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "os/os_mbuf.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "esp_central.h"

#define BT_TAG "BT_DRV"

#define BT_NAME_MAX 32

/* Nordic UART Service (NUS) - de-facto BLE "serial" bridge */
static const ble_uuid128_t bt_svc_uuid = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);
static const ble_uuid128_t bt_chr_rx_uuid = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);
static const ble_uuid128_t bt_chr_tx_uuid = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

/* Client Characteristic Configuration Descriptor (for notifications) */
static const ble_uuid16_t bt_cccd_uuid = BLE_UUID16_INIT(0x2902);

static char bt_adv_name[BT_NAME_MAX] = "EdgeTX-OpenX1";
static uint16_t bt_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t bt_tx_val_handle = 0;
static uint8_t bt_own_addr_type = BLE_OWN_ADDR_PUBLIC;
static bool bt_host_started = false;
static bool bt_host_start_ok = false;
static bool bt_host_synced = false;

/*
 * Shared ownership flag for the single NimBLE host event loop.  The
 * "BT PowerUp" module (esp_ble_powerup.c) also starts the NimBLE host;
 * only ONE host task may run nimble_port_run() at a time.  Starting a
 * second one would call ble_hs_start() again and crash the radio.
 * Declared non-static so esp_ble_powerup.c can check it.
 */
int g_nimble_host_owned = 0;

/*
 * Set by board.cpp from the return value of nimble_port_init().  If the
 * NimBLE controller/host failed to initialize at boot, starting the host
 * later (nimble_port_run -> ble_hs_start) can abort() and reboot the radio.
 * bt_host_start() checks this and backs off cleanly instead.
 */
int g_nimble_port_init_ok = 0;

/*
 * Persistent crash latch (NVS): armed just before the NimBLE host task is
 * created and cleared once the host has synced (bt_on_sync).  If the radio
 * reboots before it is cleared, the previous boot died while starting the
 * host.  Unlike esp_reset_reason(), this survives power-off/power-on cycles,
 * so a BT host-start crash can never wedge the radio in a reboot loop.
 */
#define BT_NVS_NS    "edgetx_bt"
#define BT_NVS_LATCH "bt_crash"

static bool bt_nvs_get_crash_latch(void)
{
  nvs_handle_t h;
  int32_t v = 0;
  if (nvs_open(BT_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
    nvs_get_i32(h, BT_NVS_LATCH, &v);
    nvs_close(h);
  }
  return v != 0;
}

static void bt_nvs_set_crash_latch(bool armed)
{
  nvs_handle_t h;
  if (nvs_open(BT_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
    nvs_set_i32(h, BT_NVS_LATCH, armed ? 1 : 0);
    nvs_commit(h);
    nvs_close(h);
  }
}

/* Central (master) role */
typedef enum {
  BT_ROLE_PERIPHERAL = 0,
  BT_ROLE_CENTRAL = 1
} bt_role_t;

#define BT_ADDR_STR_LEN 13 /* 12 hex chars + NUL */
#define BT_DISC_CACHE_MAX 8
#define BT_MAX_DISCOVER_DEVICES 6 /* == MAX_BLUETOOTH_DISTANT_ADDR */

static bt_role_t bt_role = BT_ROLE_PERIPHERAL;
static bool bt_scan_active = false;
static bool bt_disc_active = false;
static bool bt_peer_init_done = false;
static uint16_t bt_peer_rx_val_handle = 0; /* peer NUS RX (write target)  */
static uint16_t bt_peer_tx_val_handle = 0; /* peer NUS TX (notify source) */
static uint8_t bt_discover_count = 0;
static char bt_peer_addr[BT_ADDR_STR_LEN] = "";

/* advertised 128-bit service list (peripheral) */
static ble_uuid128_t bt_adv_uuids[1];

/* cache of NUS devices found during discovery (addr string -> type) */
typedef struct {
  char str[BT_ADDR_STR_LEN];
  uint8_t val[6];
  uint8_t type;
} bt_disc_entry_t;
static bt_disc_entry_t bt_disc_cache[BT_DISC_CACHE_MAX];
static int bt_disc_cache_len = 0;

/* ------------------------------------------------------------------ *
 * RX ring buffer (bytes received from the BLE central)               *
 * ------------------------------------------------------------------ */
#define BT_RX_RING_SIZE 1024
static uint8_t bt_rx_ring[BT_RX_RING_SIZE];
static uint32_t bt_rx_head = 0;
static uint32_t bt_rx_tail = 0;
static portMUX_TYPE bt_rx_mux = portMUX_INITIALIZER_UNLOCKED;

static void bt_rx_push(const uint8_t *data, uint32_t len)
{
  portENTER_CRITICAL(&bt_rx_mux);
  for (uint32_t i = 0; i < len; i++) {
    uint32_t next = (bt_rx_head + 1) % BT_RX_RING_SIZE;
    if (next == bt_rx_tail) break; /* full: drop */
    bt_rx_ring[bt_rx_head] = data[i];
    bt_rx_head = next;
  }
  portEXIT_CRITICAL(&bt_rx_mux);
}

static void bt_rx_push_str(const char *str)
{
  bt_rx_push((const uint8_t *)str, strlen(str));
}

static bool bt_rx_pop(uint8_t *byte)
{
  portENTER_CRITICAL(&bt_rx_mux);
  bool ok = (bt_rx_head != bt_rx_tail);
  if (ok) {
    *byte = bt_rx_ring[bt_rx_tail];
    bt_rx_tail = (bt_rx_tail + 1) % BT_RX_RING_SIZE;
  }
  portEXIT_CRITICAL(&bt_rx_mux);
  return ok;
}

/* ------------------------------------------------------------------ *
 * GATT server                                                        *
 * ------------------------------------------------------------------ */
static int bt_gap_event(struct ble_gap_event *event, void *arg);
static int bt_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def bt_svc_defs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &bt_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &bt_chr_rx_uuid.u,
                .access_cb = bt_chr_access,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &bt_chr_tx_uuid.u,
                .access_cb = bt_chr_access,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {0},
        },
    },
    {0},
};

static int bt_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg)
{
  (void)conn_handle;
  (void)attr_handle;
  (void)arg;

  if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len > 0) {
      uint8_t buf[256];
      uint16_t n = (len > sizeof(buf)) ? (uint16_t)sizeof(buf) : len;
      os_mbuf_copydata(ctxt->om, 0, n, buf);
      bt_rx_push(buf, n);
    }
    return 0;
  }

  if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
    /* Allow reads (some centrals probe the characteristics) */
    return 0;
  }

  return BLE_ATT_ERR_UNLIKELY;
}

/* ------------------------------------------------------------------ *
 * Central (master) helpers                                           *
 * ------------------------------------------------------------------ */
static void bt_addr_to_str(const uint8_t *addr, char *out)
{
  sprintf(out, "%02x%02x%02x%02x%02x%02x",
          addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
}

static bool bt_str_to_addr(const char *str, uint8_t *addr)
{
  uint8_t hi = 0;
  int nibbles = 0;
  for (const char *p = str; *p != '\0' && nibbles < 12; p++) {
    uint8_t v;
    char c = *p;
    if (c == ':') continue;
    if (c >= '0' && c <= '9') v = (uint8_t)(c - '0');
    else if (c >= 'a' && c <= 'f') v = (uint8_t)(c - 'a' + 10);
    else if (c >= 'A' && c <= 'F') v = (uint8_t)(c - 'A' + 10);
    else return false;
    if ((nibbles & 1) == 0) hi = v;
    else addr[nibbles / 2] = (uint8_t)((hi << 4) | v);
    nibbles++;
  }
  return nibbles == 12;
}

static void bt_start_scan(void)
{
  struct ble_gap_disc_params disc_params;
  int rc;

  if (bt_scan_active) return;
  bt_scan_active = true;

  memset(&disc_params, 0, sizeof(disc_params));
  disc_params.filter_duplicates = 0;
  disc_params.passive = 0;
  disc_params.limited = 0;

  rc = ble_gap_disc(bt_own_addr_type, BLE_HS_FOREVER, &disc_params,
                    bt_gap_event, NULL);
  if (rc != 0) {
    bt_scan_active = false;
    ESP_LOGW(BT_TAG, "disc start rc=%d", rc);
  }
}

static void bt_central_connect(const char *addr_str)
{
  uint8_t addr[6];
  uint8_t addr_type = BLE_ADDR_PUBLIC;
  ble_addr_t peer_addr;
  int rc;
  int i;

  if (!bt_str_to_addr(addr_str, addr)) {
    ESP_LOGW(BT_TAG, "bad peer addr '%s'", addr_str);
    return;
  }

  /* reuse the address type we saw during discovery (if any) */
  for (i = 0; i < bt_disc_cache_len; i++) {
    if (strcmp(bt_disc_cache[i].str, addr_str) == 0) {
      addr_type = bt_disc_cache[i].type;
      break;
    }
  }

  if (bt_scan_active) {
    ble_gap_disc_cancel();
    bt_scan_active = false;
  }
  bt_disc_active = false;

  rc = ble_hs_id_infer_auto(0, &bt_own_addr_type);
  if (rc != 0) {
    ESP_LOGW(BT_TAG, "id_infer_auto rc=%d", rc);
  }

  strncpy(bt_peer_addr, addr_str, BT_ADDR_STR_LEN - 1);
  bt_peer_addr[BT_ADDR_STR_LEN - 1] = '\0';

  peer_addr.type = addr_type;
  memcpy(peer_addr.val, addr, 6);

  rc = ble_gap_connect(bt_own_addr_type, &peer_addr, 30000, NULL,
                       bt_gap_event, NULL);
  if (rc != 0) {
    ESP_LOGW(BT_TAG, "connect rc=%d", rc);
  }
}

static int bt_write_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       struct ble_gatt_attr *attr, void *arg)
{
  (void)conn_handle;
  (void)error;
  (void)attr;
  (void)arg;
  return 0;
}

static void bt_central_disc_cb(const struct peer *peer, int status, void *arg)
{
  const struct peer_chr *rx;
  const struct peer_chr *tx;

  (void)arg;

  if (status != 0) {
    ESP_LOGW(BT_TAG, "peer service discovery failed status=%d", status);
    bt_rx_push_str("DisConnected\r\n");
    return;
  }

  rx = peer_chr_find_uuid(peer, &bt_svc_uuid.u, &bt_chr_rx_uuid.u);
  tx = peer_chr_find_uuid(peer, &bt_svc_uuid.u, &bt_chr_tx_uuid.u);
  if (rx == NULL || tx == NULL) {
    ESP_LOGW(BT_TAG, "peer does not expose NUS serial bridge");
    bt_rx_push_str("DisConnected\r\n");
    return;
  }

  bt_peer_rx_val_handle = rx->chr.val_handle;
  bt_peer_tx_val_handle = tx->chr.val_handle;

  /* Subscribe to the peer's TX notifications by writing the CCCD */
  {
    const struct peer_dsc *cccd =
        peer_dsc_find_uuid(peer, &bt_svc_uuid.u, &bt_chr_tx_uuid.u,
                           &bt_cccd_uuid.u);
    if (cccd != NULL) {
      uint8_t val[2] = {0x01, 0x00}; /* notifications enabled */
      ble_gattc_write_flat(peer->conn_handle, cccd->dsc.handle, val,
                           sizeof(val), bt_write_cb, NULL);
    }
  }

  /* Tell the EdgeTX state machine we are connected (as central) */
  bt_rx_push_str("Connected:");
  bt_rx_push_str(bt_peer_addr);
  bt_rx_push_str("\r\n");
}

/* ------------------------------------------------------------------ *
 * Advertising                                                        *
 * ------------------------------------------------------------------ */
static void bt_start_advertising(void)
{
  struct ble_gap_adv_params adv_params;
  struct ble_hs_adv_fields fields;
  int rc;

  memset(&fields, 0, sizeof(fields));
  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  fields.name = (uint8_t *)bt_adv_name;
  fields.name_len = (uint8_t)strlen(bt_adv_name);
  fields.name_is_complete = 1;
  /* advertise the NUS service so a central scan can find us */
  bt_adv_uuids[0] = bt_svc_uuid;
  fields.uuids128 = bt_adv_uuids;
  fields.num_uuids128 = 1;
  rc = ble_gap_adv_set_fields(&fields);
  if (rc != 0) {
    ESP_LOGW(BT_TAG, "adv_set_fields rc=%d", rc);
    return;
  }

  memset(&adv_params, 0, sizeof(adv_params));
  adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
  adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

  rc = ble_gap_adv_start(bt_own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                         bt_gap_event, NULL);
  if (rc != 0 && rc != BLE_HS_EALREADY) {
    ESP_LOGW(BT_TAG, "adv_start rc=%d", rc);
  }
}

/* ------------------------------------------------------------------ *
 * GAP event handler                                                  *
 * ------------------------------------------------------------------ */
static int bt_gap_event(struct ble_gap_event *event, void *arg)
{
  (void)arg;
  switch (event->type) {
    case BLE_GAP_EVENT_CONNECT: {
      if (event->connect.status != 0) {
        bt_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ESP_LOGW(BT_TAG, "connect failed, status=%d", event->connect.status);
        if (bt_role == BT_ROLE_PERIPHERAL) {
          bt_start_advertising();
        }
        return 0;
      }

      bt_conn_handle = event->connect.conn_handle;
      if (bt_role == BT_ROLE_CENTRAL) {
        /* we are the central: discover the peer's services */
        if (peer_add(bt_conn_handle) == 0) {
          peer_disc_all(bt_conn_handle, bt_central_disc_cb, NULL);
        }
      } else {
        /* a central connected to us (peripheral) */
        struct ble_gap_conn_desc desc;
        ble_gap_adv_stop();
        if (ble_gap_conn_find(bt_conn_handle, &desc) == 0) {
          char addr[BT_ADDR_STR_LEN];
          bt_addr_to_str(desc.peer_id_addr.val, addr);
          ESP_LOGI(BT_TAG, "central connected: %s", addr);
          bt_rx_push_str("Connected:");
          bt_rx_push_str(addr);
          bt_rx_push_str("\r\n");
        } else {
          ESP_LOGW(BT_TAG, "connected but conn_find failed");
          bt_rx_push_str("Connected:00:00:00:00:00:00\r\n");
        }
      }
      return 0;
    }

    case BLE_GAP_EVENT_DISCONNECT:
      bt_conn_handle = BLE_HS_CONN_HANDLE_NONE;
      bt_peer_rx_val_handle = 0;
      bt_peer_tx_val_handle = 0;
      ESP_LOGI(BT_TAG, "disconnected, reason=%d", event->disconnect.reason);
      peer_delete(event->disconnect.conn.conn_handle);
      if (bt_role == BT_ROLE_PERIPHERAL) {
        bt_start_advertising();
      }
      /* central: state machine retries with AT+CON on its own */
      return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
      if (bt_role == BT_ROLE_PERIPHERAL) {
        bt_start_advertising();
      }
      return 0;

    case BLE_GAP_EVENT_DISC: {
      struct ble_hs_adv_fields fields;
      int i;
      if (!bt_disc_active) return 0;
      if (event->disc.event_type != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND &&
          event->disc.event_type != BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP) {
        return 0;
      }
      if (ble_hs_adv_parse_fields(&fields, event->disc.data,
                                  event->disc.length_data) != 0) {
        return 0;
      }
      for (i = 0; i < fields.num_uuids128; i++) {
        if (ble_uuid_cmp(&fields.uuids128[i].u, &bt_svc_uuid.u) == 0) {
          char addr[BT_ADDR_STR_LEN];
          int j;
          bt_addr_to_str(event->disc.addr.val, addr);
          /* skip duplicates and cap the list */
          if (bt_discover_count >= BT_MAX_DISCOVER_DEVICES) return 0;
          for (j = 0; j < bt_disc_cache_len; j++) {
            if (strcmp(bt_disc_cache[j].str, addr) == 0) return 0;
          }
          if (bt_disc_cache_len < BT_DISC_CACHE_MAX) {
            memcpy(bt_disc_cache[bt_disc_cache_len].str, addr, BT_ADDR_STR_LEN);
            memcpy(bt_disc_cache[bt_disc_cache_len].val, event->disc.addr.val, 6);
            bt_disc_cache[bt_disc_cache_len].type = event->disc.addr.type;
            bt_disc_cache_len++;
          }
          bt_discover_count++;
          bt_rx_push_str("OK+DISC:");
          bt_rx_push_str(addr);
          bt_rx_push_str("\r\n");
          return 0;
        }
      }
      return 0;
    }

    case BLE_GAP_EVENT_DISC_COMPLETE:
      bt_scan_active = false;
      bt_disc_active = false;
      bt_rx_push_str("OK+DISCE\r\n");
      return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
      if (event->notify_rx.om) {
        uint16_t len = OS_MBUF_PKTLEN(event->notify_rx.om);
        if (len > 0) {
          uint8_t buf[256];
          uint16_t n = (len > sizeof(buf)) ? (uint16_t)sizeof(buf) : len;
          os_mbuf_copydata(event->notify_rx.om, 0, n, buf);
          bt_rx_push(buf, n);
        }
      }
      return 0;

    default:
      return 0;
  }
}

/* ------------------------------------------------------------------ *
 * AT-command synthesis (bluetooth.cpp speaks to an SPP module)       *
 * ------------------------------------------------------------------ */
static bool bt_has(const uint8_t *data, uint32_t len, const char *needle)
{
  size_t n = strlen(needle);
  if (len < n) return false;
  for (uint32_t i = 0; i + n <= len; i++) {
    if (memcmp(data + i, needle, n) == 0) return true;
  }
  return false;
}

static void bt_handle_at(const uint8_t *data, uint32_t len)
{
  if (bt_has(data, len, "AT+NAME")) {
    /* AT+NAME<name>: update the advertised BLE name */
    if (len > 7) {
      const uint8_t *name = data + 7;
      uint32_t nlen = len - 7;
      while (nlen > 0 &&
             (name[nlen - 1] == '\r' || name[nlen - 1] == '\n' ||
              name[nlen - 1] == ' ')) {
        nlen--;
      }
      if (nlen > 0 && nlen < BT_NAME_MAX) {
        memcpy(bt_adv_name, name, nlen);
        bt_adv_name[nlen] = '\0';
        if (bt_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
          ble_gap_adv_stop();
          bt_start_advertising();
        }
      }
    }
    bt_rx_push_str("OK+NAME\r\n");
  } else if (bt_has(data, len, "AT+TXPW")) {
    if (bt_role == BT_ROLE_CENTRAL) {
      bt_rx_push_str("Central:0\r\n");
    } else {
      bt_rx_push_str("Peripheral:0\r\n");
    }
  } else if (bt_has(data, len, "AT+ROLE")) {
    /* "AT+ROLE1" = central (master), "AT+ROLE0" = peripheral (slave) */
    bool is_central = false;
    uint32_t i;
    for (i = 0; i + 8 <= len; i++) {
      if (memcmp(data + i, "AT+ROLE", 7) == 0 && data[i + 7] == '1') {
        is_central = true;
        break;
      }
    }
    if (is_central) {
      bt_role = BT_ROLE_CENTRAL;
      ble_gap_adv_stop();
      bt_rx_push_str("Central:0\r\n");
    } else {
      bt_role = BT_ROLE_PERIPHERAL;
      bt_start_advertising();
      bt_rx_push_str("Peripheral:0\r\n");
    }
  } else if (bt_has(data, len, "AT+DISC?")) {
    /* master: start scanning for NUS peripherals */
    bt_disc_active = true;
    bt_discover_count = 0;
    bt_rx_push_str("OK+DISCS\r\n");
    bt_start_scan();
  } else if (bt_has(data, len, "AT+CLEAR")) {
    bt_rx_push_str("OK+CLEAR\r\n");
  } else if (bt_has(data, len, "AT+CON")) {
    /* master: connect to a specific NUS peripheral */
    if (bt_role == BT_ROLE_CENTRAL) {
      uint32_t start = 0;
      uint32_t i;
      char addr[32];
      for (i = 0; i + 6 <= len; i++) {
        if (memcmp(data + i, "AT+CON", 6) == 0) {
          start = i + 6;
          break;
        }
      }
      while (start < len && (data[start] == ' ' || data[start] == '\r' ||
                             data[start] == '\n')) {
        start++;
      }
      if (start < len) {
        uint32_t n = len - start;
        if (n > sizeof(addr) - 1) n = sizeof(addr) - 1;
        memcpy(addr, data + start, n);
        addr[n] = '\0';
        while (n > 0 && (addr[n - 1] == '\r' || addr[n - 1] == '\n' ||
                         addr[n - 1] == ' ')) {
          addr[--n] = '\0';
        }
        if (n > 0) {
          bt_central_connect(addr);
        }
      }
    }
  }
  /* "AT+BAUD..." needs no response */
}

/* ------------------------------------------------------------------ *
 * Driver interface                                                   *
 * ------------------------------------------------------------------ */
static void bt_on_reset(int reason)
{
  ESP_LOGE(BT_TAG, "NimBLE reset, reason=%d", reason);
}

static void bt_on_sync(void)
{
  int rc;

  /* Host + controller are up: clear the crash latch that was armed in
   * bt_host_start().  The next boot may safely start the host again. */
  bt_nvs_set_crash_latch(false);
  bt_host_synced = true;

  /* Never hard-assert here: a failure on first host start used to
   * abort() and reboot the radio.  Log and bail out instead so the
   * problem is visible on the serial console. */
  rc = ble_hs_util_ensure_addr(0);
  if (rc != 0) {
    ESP_LOGE(BT_TAG, "ensure_addr failed rc=%d", rc);
    return;
  }

  rc = ble_hs_id_infer_auto(0, &bt_own_addr_type);
  if (rc != 0) {
    ESP_LOGE(BT_TAG, "id_infer_auto failed rc=%d", rc);
    return;
  }

  rc = ble_gatts_count_cfg(bt_svc_defs);
  if (rc != 0) {
    ESP_LOGE(BT_TAG, "gatts_count_cfg failed rc=%d", rc);
    return;
  }
  rc = ble_gatts_add_svcs(bt_svc_defs);
  if (rc != 0) {
    ESP_LOGE(BT_TAG, "gatts_add_svcs failed rc=%d", rc);
    return;
  }
  rc = ble_gatts_start();
  if (rc != 0) {
    ESP_LOGE(BT_TAG, "gatts_start failed rc=%d", rc);
    return;
  }

  rc = ble_gatts_find_chr(&bt_svc_uuid.u, &bt_chr_tx_uuid.u, NULL,
                          &bt_tx_val_handle);
  if (rc != 0) {
    ESP_LOGW(BT_TAG, "find TX chr rc=%d", rc);
  }

  ESP_LOGI(BT_TAG, "NimBLE ready: gatts started, TX val handle=0x%04x",
           bt_tx_val_handle);
  bt_start_advertising();
}

#define BT_HOST_TASK_STACK_SIZE 4096
static void bt_host_task(void *param)
{
  (void)param;
  nimble_port_run(); /* returns only after nimble_port_stop() */
  vTaskDelete(NULL);
}

static StaticTask_t bt_host_task_tcb;
/* Keep the NimBLE host task stack in INTERNAL RAM: while this task runs,
 * NimBLE may trigger flash writes (e.g. NVS), which briefly disable the
 * PSRAM cache.  A stack in PSRAM then becomes inaccessible and causes a
 * silent cache-error panic (reset reason ESP_RST_PANIC, no panic output). */
static StackType_t bt_host_task_stack[BT_HOST_TASK_STACK_SIZE];

static void bt_host_start(void)
{
  if (bt_host_started) return;
  bt_host_started = true;
  bt_host_start_ok = false;

  /* Break boot loops.  Two independent mechanisms:
   *   1) esp_reset_reason(): a host-start crash ends as a software reset
   *      (rst:0x0c -> ESP_RST_SW) via esp_restart(), or a panic/watchdog on
   *      other paths.
   *   2) A persistent NVS crash latch (armed below, cleared in bt_on_sync):
   *      it survives power-off/power-on cycles, so the radio can never be
   *      wedged in a reboot loop by a host-start crash even across power
   *      cycles.
   * In every such case skip the host start this boot so the radio comes up
   * and the user can go change the Bluetooth mode. */
  esp_reset_reason_t bt_rst = esp_reset_reason();
  bool prev_crash =
      (bt_rst == ESP_RST_SW || bt_rst == ESP_RST_PANIC ||
       bt_rst == ESP_RST_TASK_WDT || bt_rst == ESP_RST_WDT ||
       bt_rst == ESP_RST_INT_WDT || bt_rst == ESP_RST_CPU_LOCKUP) ||
      bt_nvs_get_crash_latch();

  if (prev_crash) {
    /* Keep the latch SET: the last host start crashed and we must not try
     * again until the user explicitly turns Bluetooth off (which clears it)
     * or a host start succeeds.  This guarantees the radio can never be
     * wedged in a reboot loop, even across power-off/power-on cycles. */
    ESP_LOGE(BT_TAG,
             "previous boot crashed starting BT host (rst=%d, latch=1) - BT host start skipped this boot",
             (int)bt_rst);
    return;
  }

  /* nimble_port_init() must have succeeded at boot before we can start
   * the host; otherwise ble_hs_start() can abort and reboot the radio. */
  if (!g_nimble_port_init_ok) {
    ESP_LOGE(BT_TAG, "NimBLE not initialized at boot - BT disabled");
    return;
  }

  /* The PowerUP module may already own the NimBLE host event loop. */
  if (g_nimble_host_owned) {
    ESP_LOGW(BT_TAG, "NimBLE host already owned by another module - BT disabled");
    return;
  }
  g_nimble_host_owned = 1;

  ESP_LOGI(BT_TAG, "host start: reset reason=%d, nimble_ok=%d",
           (int)bt_rst, g_nimble_port_init_ok);

  /* Arm the crash latch BEFORE touching NimBLE: from here on, any reboot
   * means the host start crashed and the next boot must skip it.  The latch
   * is cleared in bt_on_sync() once the host is up. */
  bt_nvs_set_crash_latch(true);

  /* nimble_port_init() is already called in board.cpp at startup. */

  ble_hs_cfg.reset_cb = bt_on_reset;
  ble_hs_cfg.sync_cb = bt_on_sync;

  ble_svc_gap_init();
  ble_svc_gatt_init();
  ble_svc_gap_device_name_set(bt_adv_name);

  /* peer tracking used by the central (master) role */
  if (!bt_peer_init_done) {
    bt_peer_init_done = true;
    if (peer_init(3, 64, 64, 64) != 0) {
      ESP_LOGW(BT_TAG, "peer_init failed (may already be initialized)");
    }
  }

  xTaskCreateStaticPinnedToCore(bt_host_task, "bt_host", BT_HOST_TASK_STACK_SIZE,
                                NULL, (configMAX_PRIORITIES - 4),
                                bt_host_task_stack, &bt_host_task_tcb, 0);
  bt_host_start_ok = true;
}

void bluetoothInit(uint32_t baudrate, bool enable)
{
  (void)baudrate; /* meaningless for native BLE */

  ESP_LOGI(BT_TAG, "bluetoothInit(enable=%d)", (int)enable);

  if (!enable) {
    if (bt_scan_active) {
      ble_gap_disc_cancel();
      bt_scan_active = false;
    }
    ble_gap_adv_stop();
    /* User turned Bluetooth off: clear the crash latch so a later re-enable
     * starts from a clean slate. */
    bt_nvs_set_crash_latch(false);
    bt_host_start_ok = false;
    bt_host_synced = false;
    return;
  }

  bt_host_start();
  /* Only touch NimBLE if the host actually started AND synced this boot.
   * Calling ble_gap_adv_*() from the mixer task before the host has synced
   * (or on a host that was skipped for crash recovery) is both wrong and
   * unsafe - it can panic the radio.  bt_on_sync() starts advertising as
   * soon as the host is up, so nothing is lost. */
  if (bt_role == BT_ROLE_PERIPHERAL && bt_host_start_ok && bt_host_synced) {
    bt_start_advertising();
  }
}

void bluetoothWrite(const void *buffer, uint32_t len)
{
  const uint8_t *data = (const uint8_t *)buffer;

  /* AT commands are handled locally (synthesized responses) */
  if (len >= 2 && data[0] == 'A' && data[1] == 'T') {
    bt_handle_at(data, len);
    return;
  }

  if (bt_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
    ESP_LOGI(BT_TAG, "write len=%u dropped (no connection)", (unsigned)len);
    return;
  }

  if (bt_role == BT_ROLE_CENTRAL) {
    /* master: write to the peer's NUS RX characteristic */
    if (bt_peer_rx_val_handle != 0) {
      ble_gattc_write_flat(bt_conn_handle, bt_peer_rx_val_handle, data,
                           (uint16_t)len, bt_write_cb, NULL);
    }
  } else {
    /* slave: notify our own NUS TX characteristic */
    ESP_LOGI(BT_TAG, "write len=%u conn=%u tx_val=0x%04x",
             (unsigned)len, bt_conn_handle, bt_tx_val_handle);
    if (bt_tx_val_handle != 0) {
      struct os_mbuf *om = ble_hs_mbuf_from_flat(data, (uint16_t)len);
      if (om) {
        int rc = ble_gatts_notify_custom(bt_conn_handle, bt_tx_val_handle, om);
        ESP_LOGI(BT_TAG, "notify rc=%d", rc);
      }
    }
  }
}

int bluetoothRead(uint8_t *data)
{
  return bt_rx_pop(data) ? 1 : 0;
}

uint8_t bluetoothIsWriting(void)
{
  return 0;
}

void bluetoothDisable(void)
{
  if (bt_scan_active) {
    ble_gap_disc_cancel();
    bt_scan_active = false;
  }
  ble_gap_adv_stop();
  if (bt_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
    ble_gap_terminate(bt_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    bt_conn_handle = BLE_HS_CONN_HANDLE_NONE;
  }
  /* Bluetooth disabled by the user (Hardware -> Bluetooth -> Off): clear the
   * crash latch so a later re-enable starts from a clean slate. */
  bt_nvs_set_crash_latch(false);
  bt_host_start_ok = false;
  bt_host_synced = false;
}
