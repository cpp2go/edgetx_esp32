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
#include "simulcd.h"

#include "ff.h"

#include "hal/adc_driver.h"
#include "hal/rotary_encoder.h"
#include "hal/usb_driver.h"

#include "os/sleep.h"
#include "os/task.h"

#include "edgetx.h"
#include "debug.h"
#include "switches.h"
#include "input_mapping.h"
#include "gui/gui_common.h"
#include "mixes.h"
#if defined(GVARS)
#include "gvars.h"
#endif
#include "trainer.h"
#include "telemetry/frsky.h"
#include "telemetry/crossfire.h"
#if defined(LUA)
#include "lua/lua_api.h"
#endif

#include "gps.h"  // the radio's own position - see edgetxAndroidSetGps() below

#include <assert.h>
#include <deque>
#include <stdint.h>

// Monotonic event counter incremented each time haptic fires.
// Host (WasmSimulatorInterface) compares against last-seen value to detect new events.
uint32_t simuHapticValue = 0;

// Required stubs — called by firmware but no-ops in simulator
void hapticOn(uint32_t pwmPercent) { (void)pwmPercent; }
void hapticOff() {}

// WASM export — polled by host every 50ms to detect haptic events
uint32_t WASM_EXPORT(simuGetHaptic)() {
  return simuHapticValue;
}

// ---- The remote controller's own GPS ---------------------------------------
//
// EdgeTX has a place for a radio's own position: gpsdata_t gpsData (radio/src/gps.h), the slot
// an internal GPS module fills - and this board is built with INTERNAL_GPS on (see
// targets/tx16smk3/CMakeLists.txt), so Radio Info, the Statistics page, the top-bar GPS view
// and luaGetGPSPosition() are all in the firmware already. What is missing is the module: there
// is no GPS on a UART here.
//
// The remote controller has one of its own, so the app reads it through the DJI SDK and pushes
// it in through the function below (see DjiMsdkBridge.listenGps). Deliberately not the telemetry
// GPS: that one belongs to the aircraft, is set by the protocol drivers from what arrives over
// the RF link, and is not touched by any of this.
//
// Nothing else has to be arranged for it to be the only source: gpsWakeup() returns immediately
// while no serial driver is set. Units are the firmware's own, from gps.h - 1e-6 degrees, 0.1 m,
// 0.1 m/s, 0.1 degrees, satellites, and 0/1 for the fix.
void edgetxAndroidSetGps(int32_t latitude, int32_t longitude, int32_t altitude, uint16_t speed,
                         uint16_t course, uint8_t satellites, uint8_t fix)
{
  if (!fix) {
    // Say "no fix" rather than keep showing a position that is no longer known.
    gpsData.fix = 0;
    gpsData.numSat = 0;
    return;
  }

  gpsData.latitude = latitude;
  gpsData.longitude = longitude;
  gpsData.altitude = altitude;
  gpsData.speed = speed;
  gpsData.groundCourse = course;
  gpsData.numSat = satellites;
  gpsData.fix = 1;
  gpsData.packetCount++;
}

// Read back by the app, to tell "the position reached the firmware's own GPS" apart from "the
// app has it and the firmware does not" - the same reason edgetxAndroidGetSwitchState exists.
void edgetxAndroidGetGps(int32_t* latitude, int32_t* longitude, uint8_t* satellites, uint8_t* fix)
{
  if (latitude) *latitude = gpsData.latitude;
  if (longitude) *longitude = gpsData.longitude;
  if (satellites) *satellites = gpsData.numSat;
  if (fix) *fix = gpsData.fix;
}

int g_snapshot_idx = 0;

extern uint8_t startOptions;

char * main_thread_error = nullptr;

bool simu_shutdown = false;
bool simu_running = false;
bool simuCreateDefaultSettings = false;


volatile rotenc_t rotencValue = 0;
volatile uint32_t rotencDt = 0;

rotenc_t rotaryEncoderGetValue()
{
  return rotencValue / ROTARY_ENCODER_GRANULARITY;
}

extern const etx_hal_adc_driver_t simu_adc_driver;

void lcdCopy(void * dest, void * src);

#if defined(AUX_SERIAL) || defined(AUX2_SERIAL)
static void hostSerialInit();
#endif

void simuInit()
{
#if defined(ROTARY_ENCODER_NAVIGATION)
  rotencValue = 0;
#endif

  // Route firmware TRACE() output to host via WASM import
  traceCallback = simuTrace;

  // Init ADC driver callback
  adcInit(&simu_adc_driver);
  // Switches
  switchInit();

#if defined(AUX_SERIAL) || defined(AUX2_SERIAL)
  hostSerialInit();
#endif
}

bool keysStates[MAX_KEYS] = { false };
void simuSetKey(uint8_t key, bool state)
{
  assert(key < DIM(keysStates));
  keysStates[key] = state;
}

bool trimsStates[MAX_TRIMS * 2] = { false };
void simuSetTrim(uint8_t trim, bool state)
{
  assert(trim < DIM(trimsStates));
  trimsStates[trim] = state;
}

#if defined(SIMU_BOOTLOADER)
int bootloaderMain();
static void* bootloaderThread(void*)
{
  bootloaderMain();
  return nullptr;
}
#endif

void simuCreateDefaults()
{
  simuCreateDefaultSettings = true;
}

// Set by the host through simuSetSplashStartup(): show the boot splash and run the
// normal startup checks (the stick/switch warnings). Only the calibration wizard is
// skipped, so a settings mismatch cannot leave the radio waiting on it.
static bool s_splashStartup = false;

void simuSetSplashStartup()
{
  s_splashStartup = true;
}

// A language directory can exist while carrying almost no prompts: this port ships a
// single complete pack, and others are half-installed stubs. A usable pack holds
// dozens of prompt files, so a small count means the language has nothing to play.
static bool voicePackOnCard(const char* lang)
{
  if (lang[0] == 0 || lang[1] == 0)
    return false;

  char path[32];
  snprintf(path, sizeof(path), "/SOUNDS/%c%c", lang[0], lang[1]);

  DIR dir;
  if (f_opendir(&dir, path) != FR_OK)
    return false;

  int entries = 0;
  FILINFO fno;
  while (entries < 20 && f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != 0)
    entries++;

  f_closedir(&dir);
  return entries >= 20;
}

// Protocol selected for the internal module slot (MODULE_TYPE_*), 0 when the slot is
// off. The host logs it next to the external slot's, so a silent link can be traced to
// the slot the model actually drives.
uint8_t edgetxAndroidInternalModuleType()
{
  return g_model.moduleData[INTERNAL_MODULE].type;
}

// Protocol selected for the external module slot (MODULE_TYPE_*), 0 when the slot is off.
// The host logs it: only a serial protocol makes EdgeTX start the port, so "no bytes are
// flowing" is otherwise hard to tell from a wrong baud rate.
uint8_t edgetxAndroidExternalModuleType()
{
  return g_model.moduleData[EXTERNAL_MODULE].type;
}

// What the firmware has in RAM for the battery, so the host can show it next to its own
// reading: the warning threshold (Radio Setup -> Alarms) and the range that scales the
// percentage (Radio Setup -> Hardware). These are different settings, and only the first
// one raises the low-voltage warning. All in 100 mV steps, the range with the firmware's
// offsets removed.
void edgetxAndroidBatterySettings(uint16_t* warn, uint16_t* batMin, uint16_t* batMax)
{
  if (warn)
    *warn = g_eeGeneral.vBatWarn;
  if (batMin)
    *batMin = static_cast<uint16_t>(90 + g_eeGeneral.vBatMin);
  if (batMax)
    *batMax = static_cast<uint16_t>(120 + g_eeGeneral.vBatMax);
}

// The card this port ships carries Chinese only, while the compiled-in default is
// English. Replace that default - and only the default - so a fresh radio comes up in
// Chinese; a language the user picked themselves is left alone.
// This port drives an ELRS module from the internal module slot as well (see
// module_drivers.cpp), and the firmware's default for a model is "slot off". The
// slot's protocol is deliberately NOT forced on here: turning the internal module
// off in Model Settings has to stay off, otherwise it comes back on the next boot
// and the external module can never take over (the host drives only the enabled
// slot, see module_serial.h).
bool simuEnsureInternalModule()
{
  bool changed = false;

  // ELRS speaks CRSF at 400 kbaud. The firmware's default for a CRSF radio is stored 1,
  // which indexes CROSSFIRE_BAUDRATES to 921600 ("921k if possible", see edgetx.cpp) -
  // meant for a built-in ISRM chip, and a rate the module does not answer at. Stored 0
  // maps to 400000 once the radio is known to handle it (SPORT_MAX_BAUDRATE).
  if (g_eeGeneral.internalModuleBaudrate == 1) {
    g_eeGeneral.internalModuleBaudrate = 0;
    changed = true;
  }

  if (changed)
    TRACE("module: internal slot uses 400 kbaud");
  return changed;
}

// Battery defaults for this port: the pack is a 2S Li-ion, so 6.1 V is empty, 8.4 V is
// full and the low-voltage warning goes off at 6.6 V. Only the values the board seeded
// are replaced (BATTERY_WARN 74 = 7.4 V, BATTERY_MIN 70, BATTERY_MAX 86), so anything
// the user set themselves in Radio Setup is kept. The stored forms carry the firmware's
// offsets: vBatMin is in 100 mV steps minus 90, vBatMax minus 120.
bool simuEnsureBatteryDefaults()
{
  bool changed = false;

  if (g_eeGeneral.vBatWarn == 74) {
    g_eeGeneral.vBatWarn = 66;  // 6.6 V
    changed = true;
  }
  if (g_eeGeneral.vBatMin == 70 - 90) {
    g_eeGeneral.vBatMin = 61 - 90;  // 6.1 V
    changed = true;
  }
  if (g_eeGeneral.vBatMax == 86 - 120) {
    g_eeGeneral.vBatMax = 84 - 120;  // 8.4 V
    changed = true;
  }

  if (changed)
    TRACE("battery: using this port's defaults, warn 6.6 V, range 6.1-8.4 V");
  return changed;
}

bool simuEnsureUILanguage()
{
  if (g_eeGeneral.uiLanguage[0] != 'e' || g_eeGeneral.uiLanguage[1] != 'n')
    return false;

  if (!voicePackOnCard("cn"))
    return false;

  // Both fields are exactly two characters wide and not terminated.
  g_eeGeneral.uiLanguage[0] = 'c';
  g_eeGeneral.uiLanguage[1] = 'n';
  TRACE("ui: no English content on this card, defaulting to Chinese");
  return true;
}

bool simuEnsureVoicePack()
{
  if (voicePackOnCard(g_eeGeneral.ttsLanguage))
    return false;

  if (!voicePackOnCard(g_eeGeneral.uiLanguage))
    return false;

  // Note: both fields are exactly two characters wide and not terminated.
  g_eeGeneral.ttsLanguage[0] = g_eeGeneral.uiLanguage[0];
  g_eeGeneral.ttsLanguage[1] = g_eeGeneral.uiLanguage[1];
  TRACE("audio: no voice pack for the configured language, using the UI one");
  return true;
}

// Host time helpers used by the FatFS mapping (see simufatfs.cpp), which converts
// between a file's timestamp and the host's time_t.
bool simuLocalTime(time_t t, struct tm* result)
{
  if (result == nullptr) return false;

  struct tm* tmp = localtime(&t);
  if (tmp == nullptr) return false;

  *result = *tmp;
  return true;
}

time_t simuLocalTimeToEpoch(struct tm* tm)
{
  return tm == nullptr ? 0 : mktime(tm);
}

void simuStart(bool tests)
{
  if (simu_running)
    return;

#if !defined(COLORLCD)
  menuLevel = 0;
#endif

  if (s_splashStartup) {
    startOptions = OPENTX_START_NO_CALIBRATION;
  } else {
    startOptions = (tests ? 0 : OPENTX_START_NO_SPLASH | OPENTX_START_NO_CALIBRATION | OPENTX_START_NO_CHECKS);
  }
  simu_shutdown = false;

  /*
    g_tmr10ms must be non-zero otherwise some SF functions (that use this timer as a marker when it was last executed)
    will be executed twice on startup. Normal radio does not see this issue because g_tmr10ms is already a big number
    before the first call to the Special Functions. Not so in the simulator.

    There is another issue, some other function static variables depend on this value. If simulator is started
    multiple times in one Companion session, they are set to their initial values only first time the simulator
    is started. Therefore g_tmr10ms must also be set to non-zero value only the first time, then it must be left
    alone to continue from the previous simulator session value. See the issue #2446

  */
  if (g_tmr10ms == 0) {
    g_tmr10ms = 1;
  }

#if defined(RTCLOCK)
  time_t rawtime;
  struct tm * timeinfo;
  time (&rawtime);
  timeinfo = localtime (&rawtime);

  if (timeinfo != nullptr) {
    struct gtm gti;
    gti.tm_sec  = timeinfo->tm_sec;
    gti.tm_min  = timeinfo->tm_min;
    gti.tm_hour = timeinfo->tm_hour;
    gti.tm_mday = timeinfo->tm_mday;
    gti.tm_mon  = timeinfo->tm_mon;
    gti.tm_year = timeinfo->tm_year;
    gti.tm_wday = timeinfo->tm_wday;
    gti.tm_yday = timeinfo->tm_yday;
    g_rtcTime = gmktime(&gti);
  } else {
    g_rtcTime = rawtime;
  }
#endif

  // Init LCD callbacks
  lcdInit();

#if !defined(SIMU_BOOTLOADER)
  simuMain();
#else
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  struct sched_param sp;
  sp.sched_priority = SCHED_RR;
  pthread_attr_setschedparam(&attr, &sp);

  pthread_t bl_pid;
  pthread_create(&bl_pid, &attr, &bootloaderThread, nullptr);
#endif

  simu_running = true;
}

extern task_handle_t mixerTaskId;
extern task_handle_t menusTaskId;
#if defined(AUDIO)
extern task_handle_t audioTaskId;
#endif

void simuStop()
{
  if (!simu_running)
    return;

  simu_shutdown = true;
  task_shutdown_all();

  simu_running = false;
}

bool simuIsRunning()
{
  return simu_running;
}

bool simuLcdChanged()
{
  bool changed = simuLcdRefresh;
  simuLcdRefresh = false;
  return changed;
}

uint32_t simuLcdCopy(uint8_t* buf, uint32_t maxLen)
{
  uint32_t size = DISPLAY_BUFFER_SIZE * sizeof(pixel_t);
  if (size > maxLen) size = maxLen;
  memcpy(buf, simuLcdBuf, size);
  return size;
}

uint32_t simuLcdGetWidth()
{
  return LCD_W;
}

uint32_t simuLcdGetHeight()
{
  return LCD_H;
}

uint32_t simuLcdGetDepth()
{
#if defined(COLORLCD)
  return 16;
#elif LCD_W == 212
  return 4;
#else
  return 1;
#endif
}

#if !defined(COLORLCD)
void lcdSetRefVolt(uint8_t val)
{
}
#endif

#if LCD_W == 128
void lcdSetInvert(bool invert)
{
}
#endif

void boardInit()
{
  switchInit();
}

uint32_t pwrCheck() { return simu_shutdown ? e_power_off : e_power_on; }

bool pwrPressed() { return false; }
bool pwrOffPressed()
{
#if defined(PWR_BUTTON_PRESS)
  return pwrPressed();
#else
  return !pwrPressed();
#endif
}

void pwrInit() {}
void pwrOn() {}
void pwrOff() {}

bool UNEXPECTED_SHUTDOWN() { return false; }
void SET_POWER_REASON(uint32_t value) {}

#if defined(TRIMS_EMULATE_BUTTONS)
bool trimsAsButtons = false;

void setHatsAsKeys(bool val) { trimsAsButtons = val; }

bool getHatsAsKeys()
{
  bool lua = false;
#if defined(LUA)
  lua = isLuaStandaloneRunning();
#endif
  return (trimsAsButtons || lua);
}
#endif

void pollKeys()
{
}

uint32_t readKeys()
{
  uint32_t result = 0;

  for (int i = 0; i < MAX_KEYS; i++) {
    if (keysStates[i]) {
      result |= 1 << i;
    }
  }

  return result;
}

uint32_t readTrims()
{
  uint32_t trims = 0;

  for (int i = 0; i < keysGetMaxTrims() * 2; i++) {
    if (trimsStates[i]) {
      trims |= 1 << i;
    }
  }

  return trims;
}

int usbPlugged() { return false; }
int getSelectedUsbMode() { return USB_JOYSTICK_MODE; }
void setSelectedUsbMode(int mode) {}
void delay_ms(uint32_t ms) { }
void delay_us(uint16_t us) { }

void flashWrite(uint32_t *address, const uint32_t *buffer)
{
  sleep_ms(10);
}

uint32_t isBootloaderStart(const uint8_t * block)
{
  return 1;
}

#if defined(PCBXLITES)
bool isJackPlugged()
{
  return false;
}
#endif

void serialPrintf(const char * format, ...) { }
void serialCrlf() { }
void serialPutc(char c) { }

void boardOff()
{
}

#if defined(HAS_HARDWARE_OPTIONS)
HardwareOptions hardwareOptions;
#endif

uint32_t Master_frequency = 0;
uint32_t Current_used = 0;
uint16_t Current_max = 0;

void setSticksGain(uint8_t)
{
}

uint16_t getCurrent()
{
  return 10;
}

void calcConsumption()
{
}

void handleJackConnection() {}

int trainerModuleSbusGetByte(unsigned char*) { return 0; }

void rtcInit()
{
}

void rtcGetTime(struct gtm * t)
{
}

void rtcSetTime(const struct gtm * t)
{
}

#if defined(PCBTARANIS)
void sdPoll10ms() {}
#endif

uint32_t SD_GetCardType() { return 0; }

#if defined(USB_SERIAL)
const etx_serial_port_t UsbSerialPort = { "USB-VCP", nullptr, nullptr };
#endif

#if defined(AUX_SERIAL) || defined(AUX2_SERIAL)
#if !defined(__wasm__)
// Native builds (unit-tests, SDL simulator) don't have a WASM host to
// forward aux-serial traffic to: WASM_IMPORT is a no-op macro off-target,
// so the declarations in simulib.h become plain externs with no
// definitions.  Route them to the host sink instead (registered by the
// embedding application, see edgetxAndroidSetAuxSerialSink).  When no sink
// is registered the calls are dropped, which is how the unit-tests and the
// plain SDL simulator behave.
void simuAuxSerialStart(uint8_t port_nr, uint32_t baudrate, uint8_t encoding)
{
  edgetxAndroidAuxSerialStart(port_nr, baudrate, encoding);
}

void simuAuxSerialStop(uint8_t port_nr)
{
  edgetxAndroidAuxSerialStop(port_nr);
}

void simuAuxSerialSetBaudrate(uint8_t port_nr, uint32_t baudrate)
{
  edgetxAndroidAuxSerialSetBaudrate(port_nr, baudrate);
}

void simuAuxSerialSendBuffer(uint8_t port_nr, const uint8_t* data, uint32_t len)
{
  edgetxAndroidAuxSerialSendBuffer(port_nr, data, len);
}
#endif

// Per-port bridge state.  TX is forwarded to the host via WASM imports;
// RX bytes pushed in via simuAuxSerialReceive() are buffered in rxQueue
// and consumed by the firmware through host_drv_get_byte().
struct host_serial_port_t {
  uint8_t index;
  mutex_handle_t rxMutex;
  std::deque<uint8_t> rxQueue;
};

static host_serial_port_t hostSerialPorts[MAX_AUX_SERIAL] = {
  { SP_AUX1, {}, {} },
  { SP_AUX2, {}, {} },
};

static void hostSerialInit()
{
  for (uint8_t i = 0; i < MAX_AUX_SERIAL; ++i)
    mutex_create(&hostSerialPorts[i].rxMutex);
}

static void* host_drv_init(void* hw_def, const etx_serial_init* dev)
{
  if (hw_def == nullptr || dev == nullptr) return nullptr;

  auto* port = static_cast<host_serial_port_t*>(hw_def);
  mutex_lock(&port->rxMutex);
  port->rxQueue.clear();
  mutex_unlock(&port->rxMutex);
  simuAuxSerialStart(port->index, dev->baudrate, dev->encoding);
  return port;
}

static void host_drv_deinit(void* ctx)
{
  if (ctx == nullptr) return;
  auto* port = static_cast<host_serial_port_t*>(ctx);
  simuAuxSerialStop(port->index);
}

static void host_drv_send_byte(void* ctx, uint8_t b)
{
  if (ctx == nullptr) return;
  auto* port = static_cast<host_serial_port_t*>(ctx);
  simuAuxSerialSendBuffer(port->index, &b, 1);
}

static void host_drv_send_buffer(void* ctx, const uint8_t* b, uint32_t l)
{
  if (ctx == nullptr || b == nullptr || l == 0) return;
  auto* port = static_cast<host_serial_port_t*>(ctx);
  simuAuxSerialSendBuffer(port->index, b, l);
}

static bool host_drv_tx_completed(void*) { return true; }

static int host_drv_get_byte(void* ctx, uint8_t* b)
{
  if (ctx == nullptr || b == nullptr) return 0;
  auto* port = static_cast<host_serial_port_t*>(ctx);
  mutex_lock(&port->rxMutex);
  if (port->rxQueue.empty()) {
    mutex_unlock(&port->rxMutex);
    return 0;
  }
  *b = port->rxQueue.front();
  port->rxQueue.pop_front();
  mutex_unlock(&port->rxMutex);
  return 1;
}

static void host_drv_set_baudrate(void* ctx, uint32_t baudrate)
{
  if (ctx == nullptr) return;
  auto* port = static_cast<host_serial_port_t*>(ctx);
  simuAuxSerialSetBaudrate(port->index, baudrate);
}

const etx_serial_driver_t host_drv = {
  .init = host_drv_init,
  .deinit = host_drv_deinit,
  .sendByte = host_drv_send_byte,
  .sendBuffer = host_drv_send_buffer,
  .txCompleted = host_drv_tx_completed,
  .waitForTxCompleted = nullptr,
  .enableRx = nullptr,
  .getByte = host_drv_get_byte,
  .getLastByte = nullptr,
  .getBufferedBytes = nullptr,
  .copyRxBuffer = nullptr,
  .clearRxBuffer = nullptr,
  .getBaudrate = nullptr,
  .setBaudrate = host_drv_set_baudrate,
  .setPolarity = nullptr,
  .setHWOption = nullptr,
  .setReceiveCb = nullptr,
  .setIdleCb = nullptr,
  .setBaudrateCb = nullptr,
};

// -- RF module ports ---------------------------------------------------------
// A real radio hangs each RF module off a hardware UART, which this target does
// not have.  Each module slot is bridged to its own host serial port instead,
// so both slots can be open at the same time:
//
//   internal module (the built-in C3)  <-> host AUX1
//   external module (USB-UART adapter) <-> host AUX2
//
// Whatever protocol EdgeTX is configured to speak to a module (CRSF, SBUS,
// DSM, ...) drives that slot's driver, its TX bytes leave through
// simuAuxSerialSendBuffer() and bytes returned by the module enter the
// firmware through simuAuxSerialReceive() - each on that slot's own port
// index.  With a single port the two protocols would be interleaved on one
// wire as soon as both slots are on.
template <uint8_t PortIndex>
static void* host_mod_init(void*, const etx_serial_init* dev)
{
  auto* port = &hostSerialPorts[PortIndex];

  mutex_lock(&port->rxMutex);
  port->rxQueue.clear();
  mutex_unlock(&port->rxMutex);

  simuAuxSerialStart(port->index,
                     dev ? dev->baudrate : 115200,
                     dev ? dev->encoding : ETX_Encoding_8N1);
  return port;
}

static void host_mod_deinit(void* ctx)
{
  if (ctx == nullptr) return;
  simuAuxSerialStop(static_cast<host_serial_port_t*>(ctx)->index);
}

static void host_mod_send_byte(void* ctx, uint8_t byte)
{
  if (ctx == nullptr) return;
  simuAuxSerialSendBuffer(static_cast<host_serial_port_t*>(ctx)->index, &byte, 1);
}

static void host_mod_send_buffer(void* ctx, const uint8_t* data, uint32_t len)
{
  if (ctx == nullptr || data == nullptr || len == 0) return;
  simuAuxSerialSendBuffer(static_cast<host_serial_port_t*>(ctx)->index, data, len);
}

static bool host_mod_tx_completed(void*) { return true; }

static int host_mod_get_byte(void* ctx, uint8_t* data)
{
  return host_drv_get_byte(ctx, data);
}

static void host_mod_set_baudrate(void* ctx, uint32_t baudrate)
{
  if (ctx == nullptr) return;
  simuAuxSerialSetBaudrate(static_cast<host_serial_port_t*>(ctx)->index, baudrate);
}

// Referenced from module_drivers.cpp, hence the explicit external linkage
// (a namespace-scope const object is internal by default in C++).  Same driver
// for both slots, bound to a different host port by host_mod_init<>().
extern const etx_serial_driver_t hostModuleSerialDriver = {
  .init = host_mod_init<SP_AUX1>,
  .deinit = host_mod_deinit,
  .sendByte = host_mod_send_byte,
  .sendBuffer = host_mod_send_buffer,
  .txCompleted = host_mod_tx_completed,
  .waitForTxCompleted = nullptr,
  .enableRx = nullptr,
  .getByte = host_mod_get_byte,
  .getLastByte = nullptr,
  .getBufferedBytes = nullptr,
  .copyRxBuffer = nullptr,
  .clearRxBuffer = nullptr,
  .getBaudrate = nullptr,
  .setBaudrate = host_mod_set_baudrate,
  .setPolarity = nullptr,
  .setHWOption = nullptr,
  .setReceiveCb = nullptr,
  .setIdleCb = nullptr,
  .setBaudrateCb = nullptr,
};

// The external module's own host port (the module on the USB-UART adapter).
extern const etx_serial_driver_t hostExternalModuleSerialDriver = {
  .init = host_mod_init<SP_AUX2>,
  .deinit = host_mod_deinit,
  .sendByte = host_mod_send_byte,
  .sendBuffer = host_mod_send_buffer,
  .txCompleted = host_mod_tx_completed,
  .waitForTxCompleted = nullptr,
  .enableRx = nullptr,
  .getByte = host_mod_get_byte,
  .getLastByte = nullptr,
  .getBufferedBytes = nullptr,
  .copyRxBuffer = nullptr,
  .clearRxBuffer = nullptr,
  .getBaudrate = nullptr,
  .setBaudrate = host_mod_set_baudrate,
  .setPolarity = nullptr,
  .setHWOption = nullptr,
  .setReceiveCb = nullptr,
  .setIdleCb = nullptr,
  .setBaudrateCb = nullptr,
};

void simuAuxSerialReceive(uint8_t port_nr, const uint8_t* data, uint32_t len)
{
  if (port_nr >= MAX_AUX_SERIAL || data == nullptr || len == 0) return;
  auto& port = hostSerialPorts[port_nr];
  mutex_lock(&port.rxMutex);
  for (uint32_t i = 0; i < len; ++i)
    port.rxQueue.push_back(data[i]);
  mutex_unlock(&port.rxMutex);
}

#if defined(AUX_SERIAL_PWR_GPIO)
static void null_pwr_aux(uint8_t) {}
#endif
#endif

#if defined(AUX_SERIAL)
#if defined(AUX_SERIAL_PWR_GPIO)
  #define AUX_SERIAL_PWR null_pwr_aux
#else
  #define AUX_SERIAL_PWR nullptr
#endif
static etx_serial_port_t auxSerialPort = {
  "AUX1",
  &host_drv,
  &hostSerialPorts[SP_AUX1],
  AUX_SERIAL_PWR
};
#define AUX_SERIAL_PORT &auxSerialPort
#else
#define AUX_SERIAL_PORT nullptr
#endif

#if defined(AUX2_SERIAL)
#if defined(AUX_SERIAL_PWR_GPIO)
  #define AUX2_SERIAL_PWR null_pwr_aux
#else
  #define AUX2_SERIAL_PWR nullptr
#endif
static etx_serial_port_t aux2SerialPort = {
  "AUX2",
  &host_drv,
  &hostSerialPorts[SP_AUX2],
  AUX2_SERIAL_PWR
};
#define AUX2_SERIAL_PORT &aux2SerialPort
#else
#define AUX2_SERIAL_PORT nullptr
#endif // AUX2_SERIAL

etx_serial_port_t* serialPorts[MAX_AUX_SERIAL] = {
  AUX_SERIAL_PORT,
  AUX2_SERIAL_PORT,
};

const etx_serial_port_t* auxSerialGetPort(int port_nr)
{
  if (port_nr >= MAX_AUX_SERIAL) return nullptr;
  return serialPorts[port_nr];
}

#if defined(HARDWARE_TOUCH)
struct TouchState simTouchState = {};
bool simTouchOccured = false;

bool touchPanelInit()
{
  simTouchState.x = simTouchState.y = 0;
  return true;
}

bool touchPanelEventOccured()
{
  if(simTouchOccured)
  {
    simTouchOccured = false;
    return true;
  }
  return false;
}

void touchPanelDown(short x, short y)
{
  simTouchState.x = x;
  simTouchState.y = y;
  simTouchState.event = TE_DOWN;
  simTouchOccured = true;
}

void touchPanelUp()
{
  simTouchState.event = TE_UP;
  simTouchOccured = true;
}

void touchPanelCancel()
{
  simTouchState.event = TE_SLIDE_END;
  simTouchState.tapCount = 0;
  simTouchOccured = true;
}

struct TouchState touchPanelRead()
{
  struct TouchState st = simTouchState;
  simTouchState.deltaX = 0;
  simTouchState.deltaY = 0;
  return st;
}

struct TouchState getInternalTouchState()
{
  return simTouchState;
}
#endif

void simuTouchDown(int16_t x, int16_t y)
{
#if defined(HARDWARE_TOUCH)
  touchPanelDown(x, y);
#endif
}

void simuTouchUp()
{
#if defined(HARDWARE_TOUCH)
  touchPanelUp();
#endif
}

void simuTouchCancel()
{
#if defined(HARDWARE_TOUCH)
  touchPanelCancel();
#endif
}

void simuRotaryEncoderEvent(int32_t steps)
{
#if defined(ROTARY_ENCODER_NAVIGATION)
  // Move the encoder position. This target does have encoder navigation - the
  // TX16SMK3 hal header is generated into the build directory, so the macro is
  // set - and the firmware turns the position into LVGL encoder input: between
  // controls it moves the focus, and once a field is in edit mode it changes the
  // value (LVGL delivers LV_KEY_LEFT/RIGHT, which the controls turn into
  // EVT_ROTARY_LEFT/RIGHT themselves - see libui/numberedit.cpp).
  rotencValue += steps * ROTARY_ENCODER_GRANULARITY;

  // Time stamp for the acceleration in LvglWrapper::rotaryDriverRead(), which is
  // what makes a quick turn step further than a slow one. The real targets do
  // this from the encoder interrupt; nothing does it on the simulator, and while
  // rotencDt stays 0 every second step looks instantaneous, i.e. always
  // accelerated. Same units as the hardware drivers: milliseconds, accumulated.
  static uint16_t lastStepTick = 0;
  const uint16_t nowTick = g_tmr10ms;
  // The unsigned subtraction copes with the 10 ms counter wrapping.
  rotencDt += static_cast<uint32_t>(static_cast<uint16_t>(nowTick - lastStepTick)) * 10;
  lastStepTick = nowTick;
#endif
}

int32_t simuGetCapability(uint8_t cap)
{
  switch (cap) {
    case 0:  // CAP_LUA
#ifdef LUA
      return 1;
#else
      return 0;
#endif
    case 1:  // CAP_ROTARY_ENC
      return 0;
    case 2:  // CAP_ROTARY_ENC_NAV
#ifdef ROTARY_ENCODER_NAVIGATION
      return 1;
#else
      return 0;
#endif
    case 3:  // CAP_TELEM_FRSKY_SPORT
      return 1;
    case 4:  // CAP_SERIAL_AUX1
      return (auxSerialGetPort(SP_AUX1) != nullptr) ? 1 : 0;
    case 5:  // CAP_SERIAL_AUX2
      return (auxSerialGetPort(SP_AUX2) != nullptr) ? 1 : 0;
    default:
      return 0;
  }
}

void simuSetTrimValue(uint8_t idx, int32_t value)
{
  unsigned i = inputMappingConvertMode(idx);
  uint8_t phase = getTrimFlightMode(getFlightMode(), i);
  setTrimValue(phase, i, value);
}

void simuSendTelemetry(uint8_t module, uint8_t protocol,
                       const uint8_t* data, uint32_t len)
{
  switch (protocol) {
    case 0:  // SIMU_TELEMETRY_PROTOCOL_FRSKY_SPORT
      sportProcessTelemetryPacket(module, data, len);
      break;
    case 1:  // SIMU_TELEMETRY_PROTOCOL_FRSKY_HUB
      frskyDProcessPacket(module, data, len);
      break;
    case 2:  // SIMU_TELEMETRY_PROTOCOL_CROSSFIRE
      processCrossfireTelemetryFrame(module, (uint8_t*)data, len);
      break;
    case 3:  // SIMU_TELEMETRY_PROTOCOL_FRSKY_HUB_OOB
      if (len >= 3) {
        uint8_t id = data[0];
        int16_t value = ((uint8_t)(data[2]) << 8) + (uint8_t)(data[1]);
        processHubPacket(id, value);
      }
      break;
    default:
      break;
  }
}

void simuLuaReloadPermanentScripts()
{
#if defined(LUA)
  luaState = INTERPRETER_RELOAD_PERMANENT_SCRIPTS;
#endif
}

void simuLcdFlushed()
{
  if (simu_running)
    lcdFlushed();
}

uint8_t simuGetMaxTrainerChannels()
{
  return MAX_TRAINER_CHANNELS;
}

void simuCopyTrainerInput(const int16_t* buf, uint8_t count)
{
  if (count > MAX_TRAINER_CHANNELS)
    count = MAX_TRAINER_CHANNELS;
  for (uint8_t i = 0; i < count; i++) {
    int16_t v = buf[i];
    if (v < -512) v = -512;
    if (v > 512) v = 512;
    trainerInput[i] = v;
  }
}

void simuSetTrainerTimeout(uint16_t ms)
{
  trainerSetTimer(ms / 10);
}

// -- Output values --

uint8_t simuGetNumChannels()
{
  return MAX_OUTPUT_CHANNELS;
}

uint8_t simuCopyChannelOutputs(int16_t* buf, uint8_t maxCount)
{
  uint8_t n = MAX_OUTPUT_CHANNELS < maxCount ? MAX_OUTPUT_CHANNELS : maxCount;
  memcpy(buf, channelOutputs, n * sizeof(int16_t));
  return n;
}

uint8_t simuCopyMixOutputs(int16_t* buf, uint8_t maxCount)
{
  uint8_t n = MAX_OUTPUT_CHANNELS < maxCount ? MAX_OUTPUT_CHANNELS : maxCount;
  memcpy(buf, ex_chans, n * sizeof(int16_t));
  return n;
}

bool simuIsChannelUsed(uint8_t channel)
{
  return isChannelUsed(channel);
}

int simuGetChannelsUsed()
{
  return getChannelsUsed();
}

uint8_t simuGetMixCount()
{
  return getMixCount();
}

uint8_t simuGetNumLogicalSwitches()
{
  return MAX_LOGICAL_SWITCHES;
}

uint8_t simuCopyLogicalSwitches(uint8_t* buf, uint8_t maxCount)
{
  uint8_t n = MAX_LOGICAL_SWITCHES < maxCount ? MAX_LOGICAL_SWITCHES : maxCount;
  for (uint8_t i = 0; i < n; i++)
    buf[i] = getSwitch(SWSRC_FIRST_LOGICAL_SWITCH + i, 0) ? 1 : 0;
  return n;
}

int32_t simuGetTrimValue(uint8_t idx)
{
  uint8_t phase = getFlightMode();
  uint8_t mapped = inputMappingConvertMode(idx);
  return getTrimValue(getTrimFlightMode(phase, mapped), mapped);
}

int16_t simuGetTrimRange()
{
  return g_model.extendedTrims ? TRIM_EXTENDED_MAX : TRIM_MAX;
}

int32_t simuGetFlightMode()
{
  return getFlightMode();
}

uint8_t simuGetNumGVars()
{
#if defined(GVARS)
  return MAX_GVARS;
#else
  return 0;
#endif
}

uint8_t simuGetNumFlightModes()
{
  return MAX_FLIGHT_MODES;
}

int32_t simuGetGVar(uint8_t gv, uint8_t fm)
{
#if defined(GVARS)
  if (gv < MAX_GVARS && fm < MAX_FLIGHT_MODES) {
    uint8_t prec = g_model.gvars[gv].prec;
    uint8_t unit = g_model.gvars[gv].unit;
    int16_t value = (int16_t)GVAR_VALUE(gv, getGVarFlightMode(fm, gv));
    // Encode as gVarMode_t: value[15:0] | mode[23:16] | prec[25:24] | unit[27:26]
    return (((unit & 0x3) << 26) | ((prec & 0x3) << 24) |
            ((fm & 0xFF) << 16) | (value & 0xFFFF));
  }
#endif
  return 0;
}

bool simuGetBacklightState()
{
  return isBacklightEnabled();
}

// -- Custom (function) switches --

uint8_t simuGetNumCustomSwitches()
{
  return NUM_FUNCTIONS_SWITCHES;
}

uint8_t simuGetCustomSwitchIndex(uint8_t cfsIdx)
{
#if defined(FUNCTION_SWITCHES)
  return switchGetSwitchFromCustomIdx(cfsIdx);
#else
  (void)cfsIdx;
  return 0;
#endif
}

bool simuGetCustomSwitchState(uint8_t idx)
{
#if defined(FUNCTION_SWITCHES)
  if (idx < NUM_FUNCTIONS_SWITCHES)
    return fsLedState(idx);
#endif
  return false;
}

uint32_t simuGetCustomSwitchColor(uint8_t idx)
{
#if defined(FUNCTION_SWITCHES)
  if (idx < NUM_FUNCTIONS_SWITCHES)
    return fsGetLedRGB(idx);
#endif
  return 0;
}

#if defined(PDM_CLOCK)
#include "boards/rm-h750/pdm_software_driver.h"
void pdmStart() {}
void pdmStop() {}
bool pdmUpdateSoundLevel() { return false; }
uint8_t pdmGetSoundLevel() { return 0; }
bool pdmCapture() { return false; }
uint32_t pdmConvertToPCM(int16_t*, uint32_t) { return 0; }
#endif
