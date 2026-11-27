# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

add_definitions(-DLV_CONF_KCONFIG_EXTERNAL_INCLUDE="lv_conf_edgetx.h")
# Single framebuffer: the I80 driver copies every stripe into one shared DMA
# buffer and the flush callback blocks until it is on the wire, so a second
# PSRAM framebuffer would only cost 300K and LVGL's direct-mode buffer syncing.
add_definitions(-DESP32_SINGLE_FRAMEBUFFER)

set(EDGETX_ROOTDIR "${CMAKE_SOURCE_DIR}/../../../../..")
set(EDGETX_SRCDIR "${EDGETX_ROOTDIR}/radio/src")
set(EDGETX_TGT_CMN "${EDGETX_SRCDIR}/targets/common/esp32")

list(APPEND EXTRA_COMPONENT_DIRS "${EDGETX_SRCDIR}/thirdparty/lvgl")
list(APPEND EXTRA_COMPONENT_DIRS "${EDGETX_TGT_CMN}/components/nimble_central_utils")
list(APPEND EXTRA_COMPONENT_DIRS "${EDGETX_TGT_CMN}/components/lvgl_esp32_drivers")
add_definitions(-I${CMAKE_SOURCE_DIR} -I${EDGETX_SRCDIR} -I${EDGETX_SRCDIR}/gui/colorlcd -I${EDGETX_SRCDIR}/targets/openx1 -I${EDGETX_TGT_CMN})

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(PCB "OPENX1" CACHE STRING "Project OpenX-1")
set(TRANSLATIONS "CN" CACHE STRING "Radio language" FORCE)
option(BOOTLOADER "Include Bootloader" OFF)

# Debug aid: uncomment the next line to get the on-screen UI timing overlay and
# the UART RX hex dump back (see targets/openx1/ui_profiler.h).
#add_definitions(-DUI_FRAME_PROFILER)
unset(UI_FRAME_PROFILER CACHE)

# Single-wire (S.Port) module bay is a run-time setting: Model Setup > External
# RF > S.Port (g_eeGeneral.extModSPort), off by default.

# On-screen timing overlay (see targets/openx1/ui_profiler.h): frame rate, panel
# flush cost, and how often a standalone Lua script gets to run.
#add_definitions(-DUI_FRAME_PROFILER)

# A/B of the panel refresh path (see lcd.cpp): 0 pushes the whole screen once per
# frame, 1 pushes only the areas LVGL redrew.
add_definitions(-DLCD_FLUSH_DIRTY_AREAS=1)

# A/B of the key event handling (see keys.cpp): 0 is the original single-slot
# behaviour, 1 the event queue.
add_definitions(-DKEY_EVENT_QUEUE=0)

# A standalone Lua script is iterated once per UI frame, which on this target is
# several times longer than a module round trip. These set how many extra turns
# the script gets per frame and how far apart they are, so a script that talks to
# the module (ELRS parameter reads answer in ~5ms) can run several requests per
# frame instead of one.
#
# Disabled: the extra turns are spent unconditionally (the menus task cannot tell
# whether the script is waiting for the module), so every frame of every
# standalone script pays 4 x 5ms = 20ms of sleep plus 4 more run() calls on top of
# a 33ms frame period - the UI drops to well under 20 FPS and a drawing script
# does 5x the work with only the last iteration visible. Re-enable once the extra
# turns are gated on actual module traffic and bounded by a frame time budget.
#add_definitions(-DOPENX1_LUA_FAST_ITER=4)
#add_definitions(-DOPENX1_LUA_FAST_GAP_MS=5)

# Runtime task statistics (see targets/openx1/diag_stats.h): logs the longest
# gap/duration of the key polling, UI frame, audio task and panel flush once per
# second on the console. Uncomment when investigating lag/tearing/audio issues.
#add_definitions(-DOPENX1_DIAG_STATS)

# Temporary: log every CRSF frame we queue for Lua and every CRSF frame we send,
# with a millisecond timestamp (see pulses/crossfire.cpp and
# telemetry/crossfire.cpp). Used to see whether a Lua script is limited by our
# request rate or by the module's replies.
#add_definitions(-DOPENX1_XF_TRACE)

project(edgetx)

target_link_libraries(${project_elf} PRIVATE radiolib_esp32 board_lib)

# ESP-IDF FatFS hard-codes FF_FS_RPATH = 0 (no relative paths). Wrap the
# FatFS entry points used by Lua file/image APIs so relative paths are
# resolved against the chdir() CWD (see esp32_storage.cpp __wrap_*).
target_link_options(${project_elf} PRIVATE
  "-Wl,--wrap=f_open"
  "-Wl,--wrap=f_stat"
  "-Wl,--wrap=f_opendir"
  "-Wl,--wrap=f_unlink"
  "-Wl,--wrap=f_mkdir"
  "-Wl,--wrap=f_rename"
)
