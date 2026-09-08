/*
 * Copyright (C) OpenTX
 *
 * Based on code named
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

#ifndef _BOARD_H_
#define _BOARD_H_

#include "board_common.h"

#define auxSerialGetPort(port_nr) nullptr

//++++++++++++++++++++++++++++++++++++++++++++++++++++TODO-OPENX1

#define BACKLIGHT_LEVEL_MAX     100
#define BACKLIGHT_LEVEL_MIN     10

#define MB                              *1024*1024
#define LUA_MEM_EXTRA_MAX               (2 MB)    // max allowed memory usage for Lua bitmaps (in bytes)
#define LUA_MEM_MAX                     (6 MB)    // max allowed memory usage for complete Lua  (in bytes), 0 means unlimited

#define PERI1_FREQUENCY               30000000
#define PERI2_FREQUENCY               60000000

// Board driver
void boardInit();
void boardOff();

// TODO-OpenX1 cleanup
#define TRAINER_PPM_OUT_TASK_CORE 0
#define MIXER_TASK_CORE 0
#define PULSES_TASK_CORE 0
#define MENU_TASK_CORE 1
#define AUDIO_TASK_CORE 1

/*
From Kconfig
  LCD D0 - D7: 10, 11, 12, 13, 14, 21, 47, 48
  LCD CS    3
  LCD DC/RS 46
  LCD WR    9
  LCD LED   45

#define I2C_SCL 40
#define I2C_SDA 39

//  MOSI -1
//  MISO -1
//  RESET -1
//  SCLK -1

//  TOUCH CS -1
//  TOUCH INT 38

SPEAK_DATA  16
SPEAK_BCLK  17
SPEAK_LRCLK 18

SD_MISO 4
SD_SCK  5
SD_MOSI 6
SD_CS   7
*/

/* ---- OSPTEK ESP32-P4C5 module dev board (V1.3) pin mapping ----
 *
 * The openx1 firmware is being brought up on the OSPTEK ESP32-P4C5 dev
 * board (see osptek/esp32-p4c5-module-dev-board). Its I2C/peripheral layout
 * follows the ESP32-P4-Function-EV-Board that OSPTEK's demos are based on:
 *
 *   I2C0  : SDA = GPIO7, SCL = GPIO8  (ST7123 touch + ES8311 codec share it)
 *   Audio : MCLK = GPIO13, BCLK = GPIO12, LRCLK = GPIO10, DOUT = GPIO9,
 *           (mic) DIN = GPIO11, PA enable = GPIO53
 *   SD    : SDMMC slot0 4-bit (CLK=43 CMD=44 D0=39 D1=40 D2=41 D3=42)
 *   BL    : DSI backlight enable = GPIO20
 *   Display / Touch : MIPI-DSI dedicated pins; ST7123 on I2C0 above
 */

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_0_SCL GPIO_NUM_8
#define I2C_0_SDA GPIO_NUM_7

#define BACKLITE_PIN GPIO_NUM_20

#define USE_RMT -1
#if CONFIG_ESP_CONSOLE_UART_NUM == 0
#define FLYSKY_UART_PORT USE_RMT
#define EXTMOD_UART_PORT UART_NUM_1
#define INTMOD_UART_PORT UART_NUM_2
#else
#define FLYSKY_UART_PORT UART_NUM_0
#define EXTMOD_UART_PORT UART_NUM_1
#define INTMOD_UART_PORT UART_NUM_2
#endif

#define FLYSKY_UART_RX_PIN GPIO_NUM_41  // GIMBLE_TX
#define FLYSKY_UART_TX_PIN GPIO_NUM_42  // GIMBLE_RX

#define INTMOD_ESP_UART_TX GPIO_NUM_2 // INTMOD_RX
#define INTMOD_ESP_UART_RX GPIO_NUM_1 // INTMOD_TX

#define EXTMOD_UART_TX GPIO_NUM_15  // EXTMOD_RX
#define EXTMOD_UART_RX GPIO_NUM_8   // EXTMOD_TX

// SD card on the OSPTEK board TF slot: SDMMC slot 0, 4-bit mode.
#define SD_SDMMC_HOST 1
#define SDMMC_CLK  GPIO_NUM_43
#define SDMMC_CMD  GPIO_NUM_44
#define SDMMC_D0   GPIO_NUM_39
#define SDMMC_D1   GPIO_NUM_40
#define SDMMC_D2   GPIO_NUM_41
#define SDMMC_D3   GPIO_NUM_42

// Audio I2S / ES8311 codec (see sound driver in targets/common/esp32)
#define I2S_MCLK  GPIO_NUM_13
#define I2S_BCLK  GPIO_NUM_12
#define I2S_LRCLK GPIO_NUM_10
#define I2S_DOUT  GPIO_NUM_9
#define I2S_DIN   GPIO_NUM_11
#define I2S_AMP_EN_GPIO GPIO_NUM_53

// Bring-up: enable the ES8311 ADC (MEMS mic) capture self-test. After boot it
// logs per-channel peak/RMS every second for ~20 s so the mic can be verified.
#define OPENX1_MIC_SELFTEST

// True stereo audio output (interleaved L/R). Enables the AUDIO_STEREO code
// paths in the shared audio pipeline (stereo AudioBuffers, L/R mixing) and the
// stereo I2S slot config - the ES8311 DAC only locks its clocks cleanly on
// standard 32-BCLK-per-WS stereo I2S frames (mono framing caused hiss on top
// of the audio while playing).
//
// Sample rate stays at the EdgeTX default (32000 Hz = 8.192 MHz MCLK) exactly
// as in the 2.12 branch reference; the ES8311 clock-coefficient registers in
// board.cpp are programmed for 32 kHz / 8.192 MHz. (A 24 kHz trial left those
// registers mismatched with the I2S clock and produced no audio.)
#define AUDIO_STEREO

// PCM is sent straight to the ES8311 over standard I2S, which is signed
// two's-complement 16-bit (silence = 0x0000). Without this override the
// shared audio pipeline falls back to AUDIO_SAMPLE_FMT_U16 (silence = 0x8000)
// on non-sim/non-AUDIO_SPI targets, so playback carried a full-scale DC
// offset on the DAC -> hiss/noise on top of the audio while playing (clean
// when idle, because nothing is sent and the amp/DAC are muted).
#define AUDIO_SAMPLE_FMT AUDIO_SAMPLE_FMT_S16

#define SOFT_PWR_CTRL
uint32_t pwrCheck();
void pwrOn();
void pwrOff();
bool pwrPressed();
bool pwrOffPressed();
#define pwrForcePressed() false

void INTERNAL_MODULE_ON(void);
void INTERNAL_MODULE_OFF(void);
void INTERNAL_MODULE_BOOTCMD(uint8_t enable);
void EXTERNAL_MODULE_ON(void);
void EXTERNAL_MODULE_OFF(void);
void internal_protocol_led_on(bool on);

struct TouchState getInternalTouchState();
struct TouchState touchPanelRead();
bool touchPanelEventOccured();

#define BATTERY_WARN                  35 // 3.5V
#define BATTERY_MIN                   34 // 3.4V
#define BATTERY_MAX                   42 // 4.2V
#define BATTERY_TYPE_FIXED

void backlightInit();
#define BACKLIGHT_FORCED_ON 101
void backlightDisable();
#define BACKLIGHT_DISABLE()             backlightDisable()
void backlightEnable(uint8_t level);
#define BACKLIGHT_ENABLE()            backlightEnable(currentBacklightBright)
bool isBacklightEnabled();
void lcdFadeOut();

// Audio driver
void audioInit() ;
#if defined(OPENX1_MIC_SELFTEST)
void audioStartMicSelfTest();
#endif
#define VOLUME_LEVEL_MAX  23
#define VOLUME_LEVEL_DEF  12
#if !defined(SOFTWARE_VOLUME)
void setScaledVolume(uint8_t volume);
void setVolume(uint8_t volume);
int32_t getVolume();
#endif
void setSampleRate(uint32_t frequency);
void audioConsumeCurrentBuffer();

#define audioDisableIrq()               taskDISABLE_INTERRUPTS()
#define audioEnableIrq()                taskENABLE_INTERRUPTS()

#define hapticOff()
#define hapticOn()

// Second serial port driver
#define DEBUG_BAUDRATE                  115200
#define LUA_DEFAULT_BAUDRATE            115200

void lcdRefresh();
bool touchPanelInit(void);

void lcdInit();
void lcdInitFinish();
void lcdOff();

void lcdSetInitalFrameBuffer(void* fbAddress);

#define lcdRefreshWait()

// Top LCD driver
void toplcdInit();
void toplcdRefresh();

#if defined(CROSSFIRE)
#define TELEMETRY_FIFO_SIZE             128
#else
#define TELEMETRY_FIFO_SIZE             64
#endif

#define BATT_SCALE 1251
#define BATTERY_DIVIDER 320384
#define VOLTAGE_DROP 0

#define SLAVE_MODE()                    (g_model.trainerData.mode == TRAINER_MODE_SLAVE)

// WiFi
#ifndef ESPNOW_ETH_ALEN
#define ESPNOW_ETH_ALEN 6
#endif

void initWiFi();
void startWiFi( char *ssid_zchar, char *passwd_zchar, char* ftppass_zchar);
void stopWiFi();
const char* getWiFiStatus();
bool isWiFiStarted(uint32_t expire);

void startWiFiESPNow();
void stopWiFiESPNow();
void init_espnow();
void disable_espnow();
void pause_espnow();
void resume_espnow();
void init_bind_espnow();
void stop_bind_espnow();
bool is_binding_espnow();
void DMAInit(void);
#if defined(ROTARY_ENCODER_NAVIGATION)
void rotaryEncoderInit();
#endif

#endif // _BOARD_H_
