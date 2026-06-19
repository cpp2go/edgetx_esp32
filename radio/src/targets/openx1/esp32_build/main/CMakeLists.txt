idf_component_register(SRCS "espidf_edgetx_main.cpp"
                    INCLUDE_DIRS ""
                    REQUIRES lvgl lvgl_esp32_drivers esp_wifi esp_netif nvs_flash
                )

set(DISABLE_COMPANION YES)
set(GVARS YES)
set(LUA YES)
set(DEBUG YES)
set(CMAKE_BUILD_TYPE Release)

option(EdgeTX_SUPERBUILD "Cross-compile EdgeTX for each toolchain" OFF)

# ESP32-P4: esp_wifi_remote replaces esp_wifi sources with stubs,
# so esp_now_* symbols are unavailable. Disable ESP-NOW for P4 builds.
if("${IDF_TARGET}" STREQUAL "esp32p4")
    set(INTERNAL_MODULE_ESPNOW OFF CACHE BOOL "ESP-NOW not available on P4 via esp_wifi_remote" FORCE)
    set(EXTERNAL_MODULE_ESPNOW OFF CACHE BOOL "ESP-NOW not available on P4 via esp_wifi_remote" FORCE)
endif()

add_subdirectory(${EDGETX_ROOTDIR} edgetx)

target_link_libraries(radiolib_esp32 stdc++)
target_link_libraries(radiolib_esp32 idf::esp_adc idf::nimble_central_utils idf::lvgl_esp32_drivers idf::lvgl idf::bt idf::esp_timer idf::esp_wifi idf::driver idf::nvs_flash idf::app_update idf::sdmmc idf::esp_driver_uart idf::esp_driver_rmt idf::esp_driver_gptimer idf::esp_driver_gpio idf::esp_ringbuf idf::esp_driver_i2s idf::esp_driver_sdmmc idf::esp_driver_sdspi)

target_compile_options(radiolib_esp32 PRIVATE -I${EDGETX_TGT_CMN})
target_compile_options(radiolib_esp32 PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>)
target_compile_options(radiolib_esp32 PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wno-deprecated-enum-enum-conversion>)
target_compile_options(radiolib_esp32 PRIVATE -Wno-ignored-qualifiers -Wno-error=dangling-pointer= -Wno-error=format -Wno-format -Wno-missing-field-initializers -Wno-extra -Wno-error=overloaded-virtual -Wno-error=address-of-packed-member)

