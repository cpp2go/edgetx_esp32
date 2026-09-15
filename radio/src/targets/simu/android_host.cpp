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

#include "simulib.h"

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

// Radio battery reported by the Android app: pack voltage in mV and whether it is on
// charge. Zero mV means "unknown", in which case the drivers keep their simulated
// defaults. See adc_driver.cpp (the reading) and led_driver.cpp (the charging icon).
namespace {

volatile uint16_t s_batteryMv = 0;
volatile uint8_t s_batteryCharging = 0;

}  // namespace

void edgetxAndroidSetBattery(uint16_t millivolts, uint8_t charging)
{
  s_batteryMv = millivolts;
  s_batteryCharging = charging;
}

uint16_t edgetxAndroidBatteryMillivolts()
{
  return s_batteryMv;
}

bool edgetxAndroidIsCharging()
{
  return s_batteryCharging != 0;
}

// How much audio the Android side still owes the speaker, in bytes. The app keeps
// this up to date (see its RcAudio.java); the boot splash stays up while it is
// non-zero so the greeting is not cut off by the main view appearing.
namespace {

volatile uint32_t s_hostAudioPending = 0;

}  // namespace

void edgetxAndroidSetHostAudioPending(uint32_t bytes)
{
  s_hostAudioPending = bytes;
}

uint32_t edgetxAndroidHostAudioPending()
{
  return s_hostAudioPending;
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

// ---- Audio ------------------------------------------------------------------
// The firmware's audio mixer produces mono 16-bit PCM at AUDIO_SAMPLE_RATE (see
// audio.h and targets/simu/audio_driver.cpp) and hands it over here in buffers of
// about 10 ms. Android's AudioTrack lives in the app, so the samples go into a
// lock-free single-producer / single-consumer ring that the app drains.
//
// On overflow the incoming bytes are dropped rather than the oldest ones: reusing
// the head of the ring would mean the producer moving the consumer's index.
namespace {

// 256 KiB. The firmware can hand over a whole prompt in one burst at boot (the
// greeting is ~130 KB of 32 kHz 16-bit samples), and whatever does not fit is
// dropped - see the dropped-bytes counter the host reports.
constexpr uint32_t kAudioRingSize = 1u << 18;
uint8_t s_audioRing[kAudioRingSize];
volatile uint32_t s_audioHead = 0;  // written by the firmware's audio thread
volatile uint32_t s_audioTail = 0;  // written by the app's playback thread
volatile uint32_t s_audioDropped = 0;
volatile uint32_t s_audioWritten = 0;

}  // namespace

void simuQueueAudio(const uint8_t* buf, uint32_t len)
{
  if (buf == nullptr || len == 0) return;

  s_audioWritten += len;

  for (uint32_t i = 0; i < len; ++i) {
    const uint32_t next = (s_audioHead + 1) & (kAudioRingSize - 1);
    if (next == s_audioTail) {
      s_audioDropped += len - i;
      return;
    }
    s_audioRing[s_audioHead] = buf[i];
    s_audioHead = next;
  }
}

// Called by the app: copies out up to maxLen queued bytes, oldest first.
uint32_t edgetxAndroidTakeAudio(uint8_t* dst, uint32_t maxLen)
{
  if (dst == nullptr || maxLen == 0) return 0;

  const uint32_t head = s_audioHead;
  uint32_t len = 0;
  while (len < maxLen && s_audioTail != head) {
    dst[len++] = s_audioRing[s_audioTail];
    s_audioTail = (s_audioTail + 1) & (kAudioRingSize - 1);
  }
  return len;
}

// The app sizes its AudioTrack from this so the two cannot drift apart. Mono
// 16-bit is fixed by the firmware (audio_data_t is int16_t on the colour radios).
uint32_t edgetxAndroidAudioSampleRate()
{
  return 32000;  // AUDIO_SAMPLE_RATE, hal/audio_driver.h - no board overrides it
}

uint32_t edgetxAndroidAudioWrittenBytes() { return s_audioWritten; }
uint32_t edgetxAndroidAudioDroppedBytes() { return s_audioDropped; }

// ---- Aux serial / external RF module bridge --------------------------------
// The firmware talks to the external RF module over a serial port. There is no
// such UART here, so those bytes are handed to the app, which owns the real
// serial transport (a USB serial port on the RC). The app registers its sink
// before starting the simulator; without one every call is silently dropped.
namespace {

const edgetxAndroidSerialSink* s_auxSerialSink = nullptr;

}  // namespace

void edgetxAndroidSetAuxSerialSink(const edgetxAndroidSerialSink* sink)
{
  s_auxSerialSink = sink;
}

void edgetxAndroidAuxSerialStart(uint8_t port_nr, uint32_t baudrate, uint8_t encoding)
{
  if (s_auxSerialSink && s_auxSerialSink->start) {
    s_auxSerialSink->start(port_nr, baudrate, encoding);
  } else {
    // Otherwise this fails silently: the firmware would keep writing into a port nobody
    // is draining, and the host never learns the baud rate it should apply.
    char buf[96];
    snprintf(buf, sizeof(buf),
             "auxserial: start port %u at %u baud, but no sink is registered",
             static_cast<unsigned>(port_nr), static_cast<unsigned>(baudrate));
    simuTrace(buf);
  }
}

void edgetxAndroidAuxSerialStop(uint8_t port_nr)
{
  if (s_auxSerialSink && s_auxSerialSink->stop)
    s_auxSerialSink->stop(port_nr);
}

void edgetxAndroidAuxSerialSetBaudrate(uint8_t port_nr, uint32_t baudrate)
{
  if (s_auxSerialSink && s_auxSerialSink->setBaudrate)
    s_auxSerialSink->setBaudrate(port_nr, baudrate);
}

void edgetxAndroidAuxSerialSendBuffer(uint8_t port_nr, const uint8_t* data, uint32_t len)
{
  if (s_auxSerialSink && s_auxSerialSink->send)
    s_auxSerialSink->send(port_nr, data, len);
}
