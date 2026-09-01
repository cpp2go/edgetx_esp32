/**
 * @file disp_dsi.h
 * @brief MIPI-DSI display driver (ESP32-P4 only)
 *
 * Provides a LVGL flush callback backed by an ESP-IDF MIPI-DSI DPI panel.
 * The panel controller (ST7701 / ST7701S by default) is initialized through
 * the DSI DBI (command) channel before the DPI video stream is started.
 */

#ifndef DISP_DSI_H
#define DISP_DSI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "lvgl.h"

/* Initialize the MIPI-DSI bus, command IO and DPI panel */
void dsi_panel_init(void);

/* LVGL flush callback, copies the rendered buffer to the DPI panel */
void dsi_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area,
                       lv_color_t *color_map);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* DISP_DSI_H */
