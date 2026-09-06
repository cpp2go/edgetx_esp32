
add_definitions(-DPCB_OPENX1)

option(SHUTDOWN_CONFIRMATION "Shutdown confirmation" OFF)
option(AUTOUPDATE "Auto update internal chips from SD" OFF)

set(PWR_BUTTON "SWITCH" CACHE STRING "Pwr button type (PRESS/SWITCH)")

set(GUI_DIR colorlcd)
# UI runs at 800x480 (landscape), so use the 800x480 bitmap asset set and the
# matching large ("lrg") fonts - exactly like TX16S MK3 / stm32h7s78-dk.
# (Was 480x272: building the 800-wide UI with 480x272-sized icons + standard
# fonts made built-in icons/text metrics mismatch the layout -> things looked
# slightly offset. fonts/CMakeLists.txt maps 800x480 -> lrg automatically.)
set(BITMAPS_DIR 800x480)
add_definitions(-DCOLORLCD -DLIBOPENUI -DHARDWARE_TOUCH -DHARDWARE_KEYS)
# OpenX1 bring-up on the OSPTEK P4C5 dev board: with no keys/touch usable
# yet, the stock "Missing or bad radio data" format alert can never be
# dismissed and blocks the main menu. This auto-creates default radio
# settings whenever radio data is missing (see storageReadAll() bypass).
# Remove once keys/touch are working.
add_definitions(-DOPENX1_NO_SD_BRINGUP)
add_definitions(-DBATTGRAPH -DTHRTRACE)
set(BITMAPS_TARGET bm800_bitmaps)
set(STORAGE_MODELSLIST YES)

set(STATUS_LEDS NO)

if(DEBUG)
add_definitions(-DDEBUG)
endif()

set(FLAVOUR openx1)
set(NAVIGATION_TYPE 9x)  
set(ROTARY_ENCODER NO)
set(LUA_EXPORT lua_export_openx1)
set(BLUETOOTH NO)  
set(USB_CHARGER NO) # SW does not control the USB charger

set(SDCARD YES)
set(EEPROM EEPROM_RAW)
set(TARGET_DIR openx1)
option(PPM_LIMITS_SYMETRICAL "PPM limits symetrical" ON)
set(USB_SERIAL ON CACHE BOOL "Enable USB serial (CDC)")

if(USB_SERIAL)
  add_definitions(-DUSB_SERIAL)
endif()

add_definitions(-DREQUIRED_SDCARD_VERSION="${SDCARD_VERSION}")
add_definitions(-DLUA_INPUTS -DTIMERS -DSOFTWARE_VOLUME)
add_definitions(-DPWR_BUTTON_${PWR_BUTTON})

option(INTERNAL_MODULE_MULTI "Support for MULTI internal module" ON)
option(INTERNAL_MODULE_ELRS "Support for CRSF internal module" ON)
option(INTERNAL_MODULE_PPM "Support for PPM internal module" OFF)
option(INTERNAL_MODULE_ESPNOW "Support for ESPNOW internal module" OFF)
option(INTERNAL_MODULE_BT_POWERUP "Support for Bluetooth internal module for POWERUP" OFF)
option(EXTERNAL_MODULE_MULTI "Support for MULTI external module" ON)
option(EXTERNAL_MODULE_CRSF "Support for CRSF external module" OFF)
option(EXTERNAL_MODULE_PPM "Support for PPM external module" OFF)
option(EXTERNAL_MODULE_ESPNOW "Support for ESPNOW external module" ON)
option(EXTERNAL_MODULE_BT_POWERUP "Support for Bluetooth internal module for POWERUP" ON)

include(${RADIO_SRC_DIR}/targets/common/esp32/CMakeLists.txt)

set(TARGET_SRC
  ${TARGET_SRC}
  trainer_driver.cpp
  lcd_lvgl_esp.cpp
  backlight_driver.cpp
  ads1015_adc.cpp
  switch_driver.cpp
  module_drivers.cpp
  mcp23017_io_driver.cpp
  )

set(FIRMWARE_TARGET_SRC
  ${FIRMWARE_TARGET_SRC}
  board.cpp
  pwr_driver.cpp
  rtc_driver.cpp
  )

set(RADIO_DEPENDENCIES ${RADIO_DEPENDENCIES} ${BITMAPS_TARGET})

foreach(LANGUAGE ${TTS_LANGUAGES})
  string(TOLOWER ${LANGUAGE} lang_lower)
  set(SRC ${SRC} translations/tts/tts_${lang_lower}.cpp)
endforeach()

if (MULTIMODULE)
  add_definitions(-DMULTI_PROTOLIST)
  set(SRC ${SRC}
      io/multi_protolist.cpp
  )
include_directories(${RADIO_SRC_DIR}/fonts/colorlcd gui/${GUI_DIR} gui/${GUI_DIR}/layouts)

file(GLOB THEMES_SRC RELATIVE ${RADIO_SRC_DIR}/gui/colorlcd ${RADIO_SRC_DIR}/gui/colorlcd/themes/*.cpp)
file(GLOB LAYOUTS_SRC RELATIVE ${RADIO_SRC_DIR}/gui/colorlcd ${RADIO_SRC_DIR}/gui/colorlcd/layouts/*.cpp)
file(GLOB WIDGETS_SRC RELATIVE ${RADIO_SRC_DIR}/gui/colorlcd ${RADIO_SRC_DIR}/gui/colorlcd/widgets/*.cpp)
endif()

set(HW_DESC_JSON ${FLAVOUR}.json)
AddHWGenTarget(${HW_DESC_JSON} mcp23017_keys mcp23017_keys.inc)
AddHWGenTarget(${HW_DESC_JSON} mcp23017_switches mcp23017_switches.inc)
AddHWGenTarget(${HW_DESC_JSON} ads1015_adc_inputs ads1015_adc_inputs.inc)

# Ensure generated hardware files are produced before board_lib is built
add_custom_target(openx1_hw_defs ALL
  DEPENDS
    ${CMAKE_CURRENT_BINARY_DIR}/hal_settings.h
    ${CMAKE_CURRENT_BINARY_DIR}/hal_keys.inc
    ${CMAKE_CURRENT_BINARY_DIR}/hal_keys_lock.h
    ${CMAKE_CURRENT_BINARY_DIR}/hal_adc_inputs.inc
    ${CMAKE_CURRENT_BINARY_DIR}/mcp23017_keys.inc
    ${CMAKE_CURRENT_BINARY_DIR}/mcp23017_switches.inc
    ${CMAKE_CURRENT_BINARY_DIR}/ads1015_adc_inputs.inc
    ${CMAKE_CURRENT_BINARY_DIR}/yaml_inputs.inc
    ${CMAKE_CURRENT_BINARY_DIR}/lua_inputs.inc
    ${CMAKE_CURRENT_BINARY_DIR}/lua_mixsrc.inc
    ${CMAKE_CURRENT_BINARY_DIR}/lua_keys.inc
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_library(board_lib OBJECT EXCLUDE_FROM_ALL
  ${CMAKE_CURRENT_BINARY_DIR}/mcp23017_keys.inc
  ${CMAKE_CURRENT_BINARY_DIR}/mcp23017_switches.inc
  ${CMAKE_CURRENT_BINARY_DIR}/hal_adc_inputs.inc
  ${CMAKE_CURRENT_BINARY_DIR}/ads1015_adc_inputs.inc
  ${CMAKE_CURRENT_BINARY_DIR}/${HW_DESC_JSON}
  )

add_dependencies(board_lib openx1_hw_defs)

