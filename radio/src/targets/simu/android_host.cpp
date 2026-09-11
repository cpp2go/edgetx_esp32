/*
 * Android host layer for the EdgeTX simulator shared library.
 *
 * The SDL-free simu core (see simulib.h) is written against a small set of
 * host-provided imports. On WASI those come from JavaScript; on Android we
 * provide them here so that libedgetx-<flavour>-simulator.so is self-contained
 * and can simply be loaded by the app:
 *
 *   simuGetAnalog(idx)      ADC input value (0..4096, centre 2048)
 *   simuLcdNotify()         called by the LCD flush when a frame is ready
 *   simuQueueAudio(buf,len) audio samples produced by the firmware
 *   simuTrace(text)         firmware TRACE() output
 *
 * Note: these must keep C++ linkage - the firmware declares them without
 * extern "C" (the wasm build imports them by their exported name instead).
 *
 * License GPLv2 (same as EdgeTX).
 */

#include <android/log.h>

#include <chrono>
#include <cmath>
#include <cstdint>

#define LOG_TAG "EdgeTXSim"

// Firmware TRACE() output -> logcat.
void simuTrace(const char* text)
{
  if (text) __android_log_write(ANDROID_LOG_INFO, LOG_TAG, text);
}

// Analog inputs, mirroring what the SDL simulator feeds the firmware:
// 0..4096 with centre 2048.
//
// The Android host pushes real values read from a hardware joystick / gamepad
// (see edgetxAndroidSetAnalog below). Until it does, a slow sine keeps the
// stick/pot channels alive so the ported UI has something to display.
namespace {

constexpr uint8_t kAnalogCount = 32;

uint16_t s_analog[kAnalogCount];
volatile uint32_t s_analogValid = 0;  // bit i: the host wrote channel i
volatile uint8_t s_analogExternal = 0;

}  // namespace

// Called by libedgetx_ui.so for every joystick axis it reads.
void edgetxAndroidSetAnalog(uint8_t idx, uint16_t value)
{
  if (idx >= kAnalogCount) return;
  s_analog[idx] = value;
  s_analogValid |= (1u << idx);
}

// 0 = generate the demo sine wave, 1 = use the values pushed above.
void edgetxAndroidSetAnalogExternal(uint8_t on)
{
  s_analogExternal = on ? 1 : 0;
}

uint16_t simuGetAnalog(uint8_t idx)
{
  if (s_analogExternal) {
    if (idx < kAnalogCount && (s_analogValid & (1u << idx)))
      return s_analog[idx];
    return 2048;  // centred: the joystick does not drive this channel
  }

  using namespace std::chrono;
  const float t = duration<float>(steady_clock::now().time_since_epoch()).count();
  const float v = std::sin(t * 0.6f + static_cast<float>(idx) * 0.8f);

  int32_t value = 2048 + static_cast<int32_t>(v * 900.0f);
  if (value < 0) value = 0;
  if (value > 4096) value = 4096;
  return static_cast<uint16_t>(value);
}

// New frame available. The Android side polls simuLcdChanged()/simuLcdCopy()
// from its render loop, so there is nothing to signal here.
void simuLcdNotify() {}

// Audio is not wired up yet: drop the samples.
void simuQueueAudio(const uint8_t* buf, uint32_t len)
{
  (void)buf;
  (void)len;
}
