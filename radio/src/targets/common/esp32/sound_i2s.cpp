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

#define AUDIO_CODEC_DMA_DESC_NUM  6
#define AUDIO_CODEC_DMA_FRAME_NUM 240

static uint32_t _sampleRate = AUDIO_SAMPLE_RATE;

static i2s_chan_handle_t tx_chan;  // I2S tx channel handler

void audioInit()
{
    // Create a new channel for speaker
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.auto_clear_after_cb = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, nullptr));

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
        .sample_rate_hz = (uint32_t)_sampleRate,
        .clk_src = I2S_CLK_SRC_DEFAULT,
        .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = I2S_STD_SLOT_LEFT,
            .ws_width = I2S_DATA_BIT_WIDTH_16BIT,
            .ws_pol = false,
            .bit_shift = true,
            .left_align = true,
            .big_endian = false,
            .bit_order_lsb = false
        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK,
            .ws = I2S_LRCLK,
            .dout = I2S_DOUT,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &std_cfg));
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
    currentBuffer = (uint8_t *)buffer->data;
    currentSize = buffer->size * 2;
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
