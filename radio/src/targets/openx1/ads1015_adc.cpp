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

#include "edgetx.h"
#include "hal/adc_driver.h"
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "i2c_driver.h"
#include "ads1x15.h"
#include "hal_adc_inputs.inc"

extern i2c_master_bus_handle_t ads_i2c_bus_handle;

typedef struct {
  uint8_t ads_index;
  uint16_t mux;
  uint8_t etx_adc_channel;
} ads1015_channel_t;

#include "ads1015_adc_inputs.inc"

static TaskHandle_t s_task_handle;

uint8_t m_bitShift;   ///< bit shift amount
adsGain_t m_gain;     ///< ADC gain
uint16_t m_dataRate;  ///< Data rate

static bool ads1015_hal_adc_init() { 
    return true; 
}

// Alert thresholds only matter for the ALERT/RDY pin, which is not wired:
// program them once instead of on every single conversion.
static void ads1015_setup_thresholds(i2c_master_dev_handle_t ads)
{
  i2c_register_write_uint16(ads, ADS1X15_REG_POINTER_HITHRESH, 0x8000);
  i2c_register_write_uint16(ads, ADS1X15_REG_POINTER_LOWTHRESH, 0x0000);
}

static esp_err_t startADCReading(i2c_master_dev_handle_t ads, uint16_t mux)
{
  uint16_t config =
      ADS1X15_REG_CONFIG_CQUE_1CONV |
      ADS1X15_REG_CONFIG_CLAT_NONLAT |
      ADS1X15_REG_CONFIG_CPOL_ACTVLOW |
      ADS1X15_REG_CONFIG_CMODE_TRAD |
      ADS1X15_REG_CONFIG_MODE_SINGLE;

  config |= m_gain;
  config |= m_dataRate;
  config |= mux;
  config |= ADS1X15_REG_CONFIG_OS_SINGLE;

  esp_err_t ret = i2c_register_write_uint16(ads, ADS1X15_REG_POINTER_CONFIG, config);
  if (ret != ESP_OK) { TRACE_ERROR("ADS startADC config err=%d", (int)ret); }
  return ret;
}

static bool conversionComplete(i2c_master_dev_handle_t ads)
{
  uint16_t result = 0;
  esp_err_t ret = i2c_register_read_uint16(ads, ADS1X15_REG_POINTER_CONFIG, &result);
  if (ret != ESP_OK) return true; // treat as done to break the wait loop
  return (result & 0x8000) != 0;
}

static bool readADC(i2c_master_dev_handle_t ads, uint16_t mux, int16_t* out)
{
  if (startADCReading(ads, mux) != ESP_OK) return false;

  // One conversion takes 0.625ms at 1600 SPS: wait for it instead of polling
  // the status register back to back. This bus also carries the key expander,
  // the touch controller and the top LCD, and the polling loop used to keep it
  // busy for most of the sweep (so keys/encoder polls were delayed).
  RTOS_WAIT_MS(1);
  for (int i = 0; i < 3 && !conversionComplete(ads); i++) {
    RTOS_WAIT_MS(1);
  }

  uint16_t res = 0;
  esp_err_t ret = i2c_register_read_uint16(ads, ADS1X15_REG_POINTER_CONVERT, &res);
  if (ret != ESP_OK) { TRACE_ERROR("ADS getResult err=%d", (int)ret); return false; }

  if (m_bitShift != 0) {
    res = res >> m_bitShift;
    if (res > 0x07FF) {
      res |= 0xF000;
    }
  }
  *out = (int16_t)res;
  return true;
}

float computeVolts(int16_t counts) {
  // see data sheet Table 3
  float fsRange;
  switch (m_gain) {
  case GAIN_TWOTHIRDS:
    fsRange = 6.144f;
    break;
  case GAIN_ONE:
    fsRange = 4.096f;
    break;
  case GAIN_TWO:
    fsRange = 2.048f;
    break;
  case GAIN_FOUR:
    fsRange = 1.024f;
    break;
  case GAIN_EIGHT:
    fsRange = 0.512f;
    break;
  case GAIN_SIXTEEN:
    fsRange = 0.256f;
    break;
  default:
    fsRange = 0.0f;
  }
  return counts * (fsRange / (32768 >> m_bitShift));
}

static bool ads1015_hal_adc_start_read() { 
    return true; 
}

static void ads1015_hal_adc_wait_completion() {
}

static const etx_hal_adc_driver_t ads1015_hal_adc_driver = {
    .inputs = _hal_inputs,
    .default_pots_cfg = _pot_default_config,
    .init = ads1015_hal_adc_init,
    .start_conversion = ads1015_hal_adc_start_read,
    .wait_completion = ads1015_hal_adc_wait_completion,
};

// Idle time between two full channel sweeps (see task_adc). Keeps the shared
// I2C bus free enough for the key polls and the module power writes.
#define ADS_SWEEP_IDLE_MS 8

static void task_adc(void* pdata)
{
  s_task_handle = xTaskGetCurrentTaskHandle();
  int channel_cnt = sizeof(ads_channels) / sizeof(ads_channels[0]);
  int index = 0;
  while (1) {
    if (0xFFFF == ads_channels[index].mux) {
      // OpenX1 has no separate RTC battery — mirror main VBAT channel.
      uint8_t offset = adcGetInputOffset(ADC_INPUT_VBAT);
      uint16_t vbat = getAnalogValue(offset);
      setAnalogValue(ads_channels[index].etx_adc_channel, vbat);
    } else {
      int16_t adcval;
      if (readADC(ads[ads_channels[index].ads_index], ads_channels[index].mux, &adcval)) {
        setAnalogValue(ads_channels[index].etx_adc_channel, adcval);
      }
      // On an I2C error keep the previous sample: writing 0 would make the
      // stick jump to an end for one sweep.
    }

    index++;
    if (index >= channel_cnt) {
      // One 1ms conversion wait per channel paces the sweep at ~9ms, but this
      // bus is shared with the key expanders, the touch controller and the top
      // LCD: sweeping flat out put ~50% load on it, which cost key polls and
      // module power writes (they have no way to wait). A little idle keeps
      // every axis around 60Hz, twice what this board used to do.
      index = 0;
      RTOS_WAIT_MS(ADS_SWEEP_IDLE_MS);
    }
  }
}

#define TASKADC_STACK_SIZE (1024 * 4)
#define TASKADC_PRIO 5

static RTOS_TASK_HANDLE taskIdADC;
RTOS_DEFINE_STACK(taskIdADC, taskADC_stack, TASKADC_STACK_SIZE);
void ads1015_adc_init(void)
{
#if 1
    m_bitShift = 4;
    m_gain = GAIN_ONE; /* +/-4.096V range = Gain 1 */
    m_dataRate = RATE_ADS1015_1600SPS;
#else
    m_bitShift = 0;
    m_gain = GAIN_ONE; /* +/-4.096V range = Gain 1 */
    m_dataRate = RATE_ADS1115_128SPS;
#endif

  for (int i = 0; i < NUM_OF_ADS; i++) {
    ESP_ERROR_CHECK(i2c_master_bus_add_device(ads_i2c_bus_handle, &i2c_dev_conf[i], &ads[i]));
    ads1015_setup_thresholds(ads[i]);
  }

  adcInit(&ads1015_hal_adc_driver);

  // The stuff (POTs, VBATT) on ADS1015 are not that critical, so start a task
  // and read it in the background
  RTOS_CREATE_TASK_EX(taskIdADC, task_adc, "ADC task", taskADC_stack, TASKADC_STACK_SIZE, TASKADC_PRIO, MIXER_TASK_CORE);
}