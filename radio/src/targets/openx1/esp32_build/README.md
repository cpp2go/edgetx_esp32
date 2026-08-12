Use ESP-IDF to build EdgeTX to run on the openx1 board (ESP32-S3 or ESP32-S31).

Get source code:
`git clone --recursive https://github.com/JunOllyLi/espidf5_edgetx.git`

Build environment:
This source tree builds with ESP-IDF 6.0.1.

Select the target chip (run from this directory, `radio/src/targets/openx1/esp32_build`):

- ESP32-S3 (default openx1 layout):
  `idf.py set-target esp32s3`
- ESP32-S31:
  `idf.py set-target esp32s31`

Note: `idf.py set-target` regenerates `sdkconfig` from IDF defaults and drops the
project's custom options. There is intentionally a single committed `sdkconfig`
(no `sdkconfig.defaults`), so after switching target re-apply the custom settings
with `idf.py menuconfig` before building, then commit the regenerated `sdkconfig`:
  - PSRAM: enable SPIRAM; select the correct mode for the chip
    (ESP32-S3 = octal `SPIRAM_MODE_OCT`; ESP32-S31 = its DDR PSRAM mode) plus speed/pins.
  - Partition table: Custom, filename `partitions.csv`, offset `0x8000`.
  - LVGL LCD (`LV_TFT_*`): controller ILI9488, protocol MCU (I80), 8-bit bus,
    data/CS/DC/WR pins for the board revision.
  - Console/UART and TinyUSB options.

Also confirm the board GPIO assignments for the S31 in
`radio/src/targets/openx1/board.h` (the S31 pin block is seeded from the S3
layout and marked TODO — correct it against the S31 schematic).

Build:
`idf.py build`

Flash and monitor:
`idf.py -p <PORT> flash monitor`

System diagram:
![alt text](system_diagram.png)
