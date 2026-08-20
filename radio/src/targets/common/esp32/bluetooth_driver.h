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

#pragma once

#include <stdint.h>
#include <stdbool.h>

/*
 * ESP32-S3 native Bluetooth driver for the generic EdgeTX Bluetooth feature.
 *
 * The ESP32-S3 only supports Bluetooth LE (no classic BR/EDR SPP), so instead
 * of talking to an external AT-command serial module this driver exposes a
 * Nordic UART Service (NUS) BLE bridge.  The EdgeTX bluetooth.cpp state
 * machine still sends its usual "AT+..." commands; this driver intercepts
 * them and synthesizes the expected responses, then bridges the trainer /
 * telemetry byte stream over BLE.
 *
 * See bluetooth_driver.c for details.
 */

#define BLUETOOTH_BOOTLOADER_BAUDRATE   230400
#define BLUETOOTH_DEFAULT_BAUDRATE      115200
#define BLUETOOTH_FACTORY_BAUDRATE      57600

#define BT_TX_FIFO_SIZE    64
#define BT_RX_FIFO_SIZE    256

/* The implementation (bluetooth_driver.c) is compiled as C, while the shared
 * bluetooth.cpp is C++.  Declare the interface with C linkage so the C++ side
 * finds the C symbols at link time. */
#ifdef __cplusplus
extern "C" {
#endif

void bluetoothInit(uint32_t baudrate, bool enable);
void bluetoothWrite(const void *buffer, uint32_t length);
int bluetoothRead(uint8_t *data);
uint8_t bluetoothIsWriting();
void bluetoothDisable();

#ifdef __cplusplus
}
#endif

// The ESP32-S3 always has its native Bluetooth radio.
#define IS_BLUETOOTH_CHIP_PRESENT()     (true)
