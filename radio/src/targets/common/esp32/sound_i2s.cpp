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

#include <driver/i2s_std.h>

#include "edgetx.h"

#if defined(I2S_AMP_EN_GPIO)
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#define AUDIO_CODEC_DMA_DESC_NUM  6
#define AUDIO_CODEC_DMA_FRAME_NUM 240

static uint32_t _sampleRate = AUDIO_SAMPLE_RATE;

static i2s_chan_handle_t tx_chan;  // I2S tx channel handler
static i2s_chan_handle_t rx_chan;  // I2S rx channel handler (ES8311 ADC / mic)

void audioInit()
{
    // Create a full-duplex channel for the ES8311 codec
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.auto_clear_after_cb = true;
#if defined(I2S_DIN)
    // RX captures the ES8311 ADC (MEMS mic) on I2S_DIN.
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, &rx_chan));
#else
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, nullptr));
#endif

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
        .sample_rate_hz = (uint32_t)_sampleRate,
        .clk_src = I2S_CLK_SRC_DEFAULT,
        .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
#if defined(AUDIO_STEREO)
            // ES8311 expects standard stereo I2S frames (32 BCLK per WS).
            // Mono mode frames the data differently and the DAC does not lock
            // its clocks cleanly -> hiss on top of the audio while playing.
            .slot_mode = I2S_SLOT_MODE_STEREO,
            .slot_mask = I2S_STD_SLOT_BOTH,
#else
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = I2S_STD_SLOT_LEFT,
#endif
            .ws_width = I2S_DATA_BIT_WIDTH_16BIT,
            .ws_pol = false,
            .bit_shift = true,
            .left_align = true,
            .big_endian = false,
            .bit_order_lsb = false
        },
        .gpio_cfg = {
#if defined(I2S_MCLK)
            .mclk = I2S_MCLK,
#else
            .mclk = I2S_GPIO_UNUSED,
#endif
            .bclk = I2S_BCLK,
            .ws = I2S_LRCLK,
            .dout = I2S_DOUT,
#if defined(I2S_DIN)
            .din = I2S_DIN,
#else
            .din = I2S_GPIO_UNUSED,
#endif
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &std_cfg));
#if defined(I2S_DIN)
    // Same clock/slot framing on RX; the codec ADC data (ES8311 SDOUT) arrives
    // on I2S_DIN once es8311AudioInit() has powered up the ADC path.
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
#endif
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
}

void setSampleRate(uint32_t frequency)
{
  const i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(frequency);

  i2s_channel_disable(tx_chan);
  i2s_channel_reconfig_std_clock(tx_chan, &clk_cfg);
  i2s_channel_enable(tx_chan);
}

static uint8_t *currentBuffer = nullptr;
static uint32_t currentSize = 0;

void audioSetCurrentBuffer(const AudioBuffer *buffer)
{
  if (buffer) {
#if defined(AUDIO_STEREO)
    // AudioBuffer holds interleaved L/R frames; each frame = 2 channels * 2 bytes.
    currentBuffer = (uint8_t *)buffer->data;
    currentSize = buffer->size * 4;  // frames * 2 channels * 2 bytes
#else
    currentBuffer = (uint8_t *)buffer->data;
    currentSize = buffer->size * 2;
#endif
  } else {
    currentBuffer = nullptr;
    currentSize = 0;
  }
}

static bool channel_enabled = true;

void audioConsumeCurrentBuffer()
{
  if (!currentBuffer) {
    audioSetCurrentBuffer(audioQueue.buffersFifo.getNextFilledBuffer());
  }

  const bool hasAudio = (currentBuffer && currentSize);
  (void)hasAudio;

  static size_t last = 0U;
  if ((NULL == currentBuffer) && (0U != last)) {
    // end of transfer?
    last = 0U;
    i2s_channel_disable(tx_chan);
    channel_enabled = false;
  }

  while (currentBuffer && currentSize) {
    if (!channel_enabled) {
      channel_enabled = true;
      i2s_channel_enable(tx_chan);
    }

    size_t written = 0U;
    i2s_channel_write(tx_chan, currentBuffer, currentSize, &written, 1000);
    last = written;

    if (written > currentSize) 
        written = currentSize;

    currentBuffer += written;
    currentSize -= written;
    if (currentSize == 0) {
      audioQueue.buffersFifo.freeNextFilledBuffer();
      currentBuffer = nullptr;
      currentSize = 0;
    }
  }
}

#if defined(OPENX1_MIC_SELFTEST)
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *MIC_TAG = "MIC";

// Bring-up helper: every second it reads ~100 ms of ES8311 ADC (mic) frames
// and logs per-channel peak/RMS. Speak or blow into the MSM381A mic -> RMS
// rises; silent -> near 0. Self-deletes after runCount iterations.
static void audioMicProbeTask(void *arg)
{
    const int runCount = (int)(intptr_t)arg;
    const size_t bytes = 3200 * 2 * sizeof(int16_t);   // 100 ms @ 32 kHz stereo
    int16_t *buf = (int16_t *)malloc(bytes);
    if (buf) {
        for (int r = 0; r < runCount; r++) {
            size_t got = 0;
            esp_err_t err = i2s_channel_read(rx_chan, buf, bytes, &got, 1000);
            if (err != ESP_OK) {
                ESP_LOGW(MIC_TAG, "read failed: %s", esp_err_to_name(err));
            } else {
                size_t n = got / sizeof(int16_t);
                int64_t accL = 0, accR = 0;
                int32_t peakL = 0, peakR = 0;
                size_t cntL = 0, cntR = 0;
                for (size_t i = 0; i + 1 < n; i += 2) {
                    int32_t sL = buf[i];
                    int32_t sR = buf[i + 1];
                    if (sL < 0) sL = -sL;
                    if (sR < 0) sR = -sR;
                    if (sL > peakL) peakL = sL;
                    if (sR > peakR) peakR = sR;
                    accL += (int64_t)buf[i] * buf[i];
                    accR += (int64_t)buf[i + 1] * buf[i + 1];
                    cntL++;
                    cntR++;
                }
                uint32_t rmsL = cntL ? (uint32_t)(sqrt((double)accL / cntL) + 0.5) : 0;
                uint32_t rmsR = cntR ? (uint32_t)(sqrt((double)accR / cntR) + 0.5) : 0;
                ESP_LOGI(MIC_TAG, "frames=%u L:peak=%u rms=%u  R:peak=%u rms=%u",
                         (unsigned)(n / 2), peakL, rmsL, peakR, rmsR);
            }
            vTaskDelay(pdMS_TO_TICKS(900));
        }
        free(buf);
    }
    vTaskDelete(NULL);
}

void audioStartMicSelfTest()
{
    static bool started = false;
    if (started || !rx_chan) return;
    started = true;
    xTaskCreate(audioMicProbeTask, "micProbe", 4096, (void *)20, 5, nullptr);
}
#endif

