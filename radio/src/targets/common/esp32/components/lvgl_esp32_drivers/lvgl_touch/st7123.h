/*
 * Sitronix ST7123 capacitive touch controller driver (I2C).
 *
 * Used on OSPTEK YDP430BT009-V1 (4.3" 480x800 MIPI-DSI) touch modules.
 * The controller is addressed through 16-bit register numbers and reports
 * its coordinates in the report table. Register map follows the ESP-IDF
 * `esp_lcd_touch_st7123` component.
 *
 * This driver plugs into the LVGL ESP32 drivers touch layer (see
 * touch_driver.c) and talks over the shared `lvgl_i2c_bus_handle` bus.
 */

#ifndef __ST7123_H
#define __ST7123_H

#include <stdint.h>
#include <stdbool.h>

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define ST7123_I2C_SLAVE_ADDR   0x55

/* Register map (16-bit register addresses, big-endian over I2C).
 * Matches the official Espressif esp_lcd_touch_st7123 component. */
#define ST7123_FW_VERSION_REG       0x0000
#define ST7123_FW_REVISION_REG      0x000C
#define ST7123_MAX_X_COORD_H_REG    0x0005   /* +1 x_l, +2 y_h, +3 y_l */
#define ST7123_MAX_Y_COORD_H_REG    0x0007
#define ST7123_MAX_TOUCHES_REG      0x0009
#define ST7123_ADV_INFO_REG         0x0010   /* advanced-info (bit3 = with_coord) */
#define ST7123_REPORT_COORD_0_REG   0x0014   /* touch report entries (7 bytes each) */
#define ST7123_ADV_INFO_WITH_COORD  (1 << 3)

#define ST7123_MAX_TOUCHES          10
#define ST7123_TOUCH_REPORT_BYTES   7

/**
 * @brief Initialize communication with the ST7123 controller.
 * @param dev_addr 7-bit I2C address of the controller (ST7123_I2C_SLAVE_ADDR).
 */
void st7123_init(uint16_t dev_addr);

/**
 * @brief Read the touch panel state.
 * @param drv LVGL indev driver (may be NULL).
 * @param data LVGL indev data filled with coordinates / state.
 * @return Always false (no continuous read requested).
 */
bool st7123_read(lv_indev_drv_t *drv, lv_indev_data_t *data);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __ST7123_H */
