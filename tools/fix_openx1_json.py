#!/usr/bin/env python3
"""Fix openx1.json to match updated Pydantic schema."""

import json

JSON_PATH = "f:/dev/keilc/opentx/edgetx_esp32/radio/src/boards/hw_defs/openx1.json"

with open(JSON_PATH, 'r') as f:
    data = json.load(f)

# Fix ADC entries: sample_time and adc must be strings
for adc in data.get('adc_inputs', {}).get('adcs', []):
    if adc.get('sample_time') is not None and not isinstance(adc['sample_time'], str):
        adc['sample_time'] = str(adc['sample_time'])
    if adc.get('adc') is not None and not isinstance(adc['adc'], str):
        adc['adc'] = str(adc['adc'])

# Fix input RTCBatInput channel to string
for inp in data.get('adc_inputs', {}).get('inputs', []):
    if inp.get('type') == 'RTC_BAT' and inp.get('channel') is not None:
        inp['channel'] = str(inp['channel'])

# Fix switches: gpio, pin, gpio_high, pin_high, gpio_low, pin_low to strings
for sw in data.get('switches', []):
    for field in ['gpio', 'pin', 'gpio_high', 'pin_high', 'gpio_low', 'pin_low']:
        if sw.get(field) is not None and not isinstance(sw[field], str):
            sw[field] = str(sw[field])

# Fix keys: gpio, pin to strings
for key in data.get('keys', []):
    for field in ['gpio', 'pin']:
        if key.get(field) is not None and not isinstance(key[field], str):
            key[field] = str(key[field])

# Fix trims: dec/inc gpio, pin to strings
for trim in data.get('trims', []):
    for sub in ['dec', 'inc']:
        if sub in trim:
            for field in ['gpio', 'pin']:
                if trim[sub].get(field) is not None and not isinstance(trim[sub][field], str):
                    trim[sub][field] = str(trim[sub][field])

# Fix display field names: old schema used w/h/phys_w/phys_h/depth
# new schema uses lcd_w/lcd_h/lcd_phys_w/lcd_phys_h/lcd_depth
old_display = data.get('display', {})
if old_display:
    new_display = {}
    mapping = {
        'w': 'lcd_w',
        'h': 'lcd_h',
        'phys_w': 'lcd_phys_w',
        'phys_h': 'lcd_phys_h',
        'depth': 'lcd_depth',
    }
    for old_key, new_key in mapping.items():
        if old_key in old_display:
            new_display[new_key] = old_display[old_key]
    # Copy over any other optional fields
    for k in ['oled_screen', 'lcd_horizontal_invert', 'lcd_vertical_invert',
              'ltdc_irq_prio', 'dma_screen_irq_prio']:
        if k in old_display:
            new_display[k] = old_display[k]
    data['display'] = new_display

# Add timers section for ESP32 (dummy/safe values - not used on ESP32)
# ESP32 uses its own timer infrastructure via FreeRTOS/timers_driver.cpp
if 'timers' not in data:
    data['timers'] = {
        "cpu_freq": 240000000,
        "peri1_frequency": 80000000,
        "peri2_frequency": 80000000,
        "timer_mult_apb1": 2,
        "timer_mult_apb2": 2,
        "ms_timer": "N/A",
        "ms_timer_irqn": "N/A",
        "ms_timer_irqhandler": "N/A",
        "mixer_scheduler_timer": "N/A",
        "mixer_scheduler_timer_freq": "N/A",
        "mixer_scheduler_timer_irqn": "N/A",
        "mixer_scheduler_timer_irqhandler": "N/A"
    }

with open(JSON_PATH, 'w') as f:
    json.dump(data, f, indent=2)

print("Fixed openx1.json successfully!")
