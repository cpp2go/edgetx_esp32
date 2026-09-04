/**
 * @file disp_dsi.c
 * @brief MIPI-DSI display driver (ESP32-P4 only)
 *
 * This driver brings up a MIPI-DSI panel using the ESP-IDF esp_lcd MIPI-DSI
 * API (esp_lcd_mipi_dsi.h). It is self contained:
 *   - powers the MIPI DSI PHY through an internal LDO regulator
 *   - creates the DSI bus and a DBI (command) panel IO
 *   - creates a DPI (video mode) panel using the timing from menuconfig
 *   - sends the panel controller init sequence over the DBI channel
 *   - provides a LVGL flush callback that copies the frame into the panel
 *
 * The controller is selected from menuconfig: ST7701 / ST7701S (default) or
 * ST7102 (OSPTEK 4.3" 480x800 modules). The init sequence and the video
 * timing are board specific: please adjust them (and/or menuconfig) to match
 * your exact panel module.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_ldo_regulator.h"
#include "lvgl.h"

#include "disp_dsi.h"

static const char *TAG = "lcd-dsi";

/* ------------------------------------------------------------------------ */
/* DSI / DPI configuration from menuconfig                                   */
/* ------------------------------------------------------------------------ */

#define DSI_LANE_NUM       CONFIG_LV_TFT_DSI_LANE_NUM
#define DSI_LANE_BITRATE   CONFIG_LV_TFT_DSI_LANE_BITRATE_MBPS
#define DSI_PHY_LDO_CHAN   CONFIG_LV_TFT_DSI_PHY_LDO_CHAN
#define DSI_PHY_LDO_MV     CONFIG_LV_TFT_DSI_PHY_LDO_VOLTAGE_MV
#define DSI_PANEL_RST      CONFIG_LV_TFT_DSI_PANEL_RST_NUM

#define DPI_CLK_MHZ        CONFIG_LV_TFT_DSI_DPI_CLK_MHZ
#define DSI_H_RES          CONFIG_LV_TFT_DSI_H_RES
#define DSI_V_RES          CONFIG_LV_TFT_DSI_V_RES
#define DSI_HSYNC          CONFIG_LV_TFT_DSI_HSYNC
#define DSI_HBP            CONFIG_LV_TFT_DSI_HBP
#define DSI_HFP            CONFIG_LV_TFT_DSI_HFP
#define DSI_VSYNC          CONFIG_LV_TFT_DSI_VSYNC
#define DSI_VBP            CONFIG_LV_TFT_DSI_VBP
#define DSI_VFP            CONFIG_LV_TFT_DSI_VFP

#if defined(CONFIG_LV_TFT_DSI_RGB565_TO_RGB888)
#define DSI_IN_COLOR_FMT   LCD_COLOR_FMT_RGB565
#define DSI_OUT_COLOR_FMT  LCD_COLOR_FMT_RGB888
#else
#define DSI_IN_COLOR_FMT   LCD_COLOR_FMT_RGB565
#define DSI_OUT_COLOR_FMT  LCD_COLOR_FMT_RGB565
#endif

/* ------------------------------------------------------------------------ */
/* ST7701 / ST7701S init sequence                                           */
/* ------------------------------------------------------------------------ */

typedef struct {
    uint8_t cmd;              /* DCS command byte */
    const uint8_t *data;      /* command parameters (NULL when len == 0) */
    uint8_t len;              /* number of parameters */
    uint32_t delay_ms;        /* delay after the command (0 = none) */
} dsi_init_cmd_t;

#if !defined(CONFIG_LV_TFT_DISPLAY_CONTROLLER_ST7102)
/* Default init sequence for a generic ST7701S 320x480 MIPI-DSI panel.
 * NOTE: register values are panel vendor specific, adjust them to the exact
 * module you are using (they control scan direction, porch, VCOM, gamma...).
 */
static const dsi_init_cmd_t st7701s_init[] = {
    // Enable manufacturer command access
    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x10}, 5, 0},
    // Line setting
    {0xC0, (uint8_t[]){0x3B, 0x00}, 2, 0},
    // Porch setting
    {0xC1, (uint8_t[]){0x0D, 0x02}, 2, 0},
    // Porch setting
    {0xC2, (uint8_t[]){0x31, 0x05}, 2, 0},
    // Gate setting
    {0xC3, (uint8_t[]){0x01, 0x00}, 2, 0},
    // Source setting
    {0xC4, (uint8_t[]){0x31, 0x00}, 2, 0},
    // MIPI setting (lane count / speed, panel specific)
    {0xC5, (uint8_t[]){0x00, 0x02}, 2, 0},
    // VCOM setting
    {0xC6, (uint8_t[]){0x1D, 0x00}, 2, 0},
    // Vendor setting
    {0xC7, (uint8_t[]){0x10}, 1, 0},
    // Vendor setting
    {0xC8, (uint8_t[]){0x03, 0x00}, 2, 0},
    // Power setting
    {0xD0, (uint8_t[]){0x9D, 0x0D}, 2, 0},
    // VCOM setting
    {0xD1, (uint8_t[]){0x31, 0x15}, 2, 0},
    // Power setting
    {0xD2, (uint8_t[]){0x22, 0x02}, 2, 0},
    // Vendor setting
    {0xD3, (uint8_t[]){0x08, 0x00}, 2, 0},
    // Vendor setting
    {0xD4, (uint8_t[]){0x6D}, 1, 0},
    // Vendor setting
    {0xD5, (uint8_t[]){0x26, 0x06}, 2, 0},
    // Gamma positive
    {0xE0, (uint8_t[]){0x00, 0x20, 0x00}, 3, 0},
    // Gamma negative
    {0xE1, (uint8_t[]){0x00, 0x20, 0x00}, 3, 0},
    // Disable manufacturer command access
    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x11}, 5, 0},
    // Sleep out
    {0x11, NULL, 0, 120},
    // Display on
    {0x29, NULL, 0, 20},
};
#endif /* !CONFIG_LV_TFT_DISPLAY_CONTROLLER_ST7102 */

#if defined(CONFIG_LV_TFT_DISPLAY_CONTROLLER_ST7102)
/* OSPTEK YDP430BT009-V1 4.3" 480x800 MIPI-DSI panel (ST7102, BOE glass).
 * This is the module vendor's release init sequence (see OSPTEK
 * "4.3-tft-480x800-mipi-st7102"). The module has no RESET pin, so a software
 * reset is issued first; command page 1, MADCTL (normal orientation) and
 * COLMOD (RGB565) are programmed before the vendor-specific list.
 */
static const dsi_init_cmd_t st7102_init[] = {
    // Software reset
    {0x01, NULL, 0, 120},
    // Select command page 1
    {0xF0, (uint8_t[]){0x00}, 1, 0},
    // Memory data access control: normal orientation
    {0x36, (uint8_t[]){0x00}, 1, 0},
    // Interface pixel format: RGB565
    {0x3A, (uint8_t[]){0x55}, 1, 0},
    // --- vendor init sequence (GX09C + BOE 4.3", 2-lane) ---
    {0x99, (uint8_t[]){0x71,0x02,0xa2}, 3, 0},
    {0x99, (uint8_t[]){0x71,0x02,0xa3}, 3, 0},
    {0x99, (uint8_t[]){0x71,0x02,0xa4}, 3, 0},
    {0xB0, (uint8_t[]){0x22,0x57,0x1E,0x61,0x2F,0x57,0x61}, 7, 0},
    {0xB7, (uint8_t[]){0x64,0x64}, 2, 0},
    {0xBF, (uint8_t[]){0xB4,0xB4}, 2, 0},
    {0xC8, (uint8_t[]){0x00,0x00,0x13,0x24,0x44,0x00,0x74,0x03,0xB8,0x04,
                        0x11,0x16,0x08,0x86,0x04,0x21,0xD3,0x02,0x10,0x0F,
                        0x22,0x4D,0x0E,0x90,0x09,0x32,0xF0,0x0B,0x40,0x0E,
                        0xF3,0x7D,0x0E,0xA9,0xBF,0x03,0xC4}, 37, 0},
    {0xC9, (uint8_t[]){0x00,0x00,0x13,0x24,0x44,0x00,0x74,0x03,0xB8,0x04,
                        0x11,0x16,0x08,0x86,0x04,0x21,0xD3,0x02,0x10,0x0F,
                        0x22,0x4D,0x0E,0x90,0x09,0x32,0xF0,0x0B,0x40,0x0E,
                        0xF3,0x7D,0x0E,0xA9,0xBF,0x03,0xC4}, 37, 0},
    {0xD7, (uint8_t[]){0x10,0x0C,0x36,0x19,0x90,0x90}, 6, 0},
    {0xA3, (uint8_t[]){0x51,0x03,0x80,0xCF,0x44,0x00,0x00,0x00,0x00,0x04,
                        0x78,0x78,0x00,0x1A,0x00,0x45,0x05,0x00,0x00,0x00,
                        0x00,0x46,0x00,0x00,0x02,0x20,0x52,0x00,0x05,0x00,
                        0x00,0xFF}, 32, 0},
    {0xA6, (uint8_t[]){0x02,0x00,0x24,0x55,0x35,0x00,0x38,0x00,0x78,0x78,
                        0x00,0x24,0x55,0x36,0x00,0x37,0x00,0x78,0x78,0x02,
                        0xAC,0x51,0x3A,0x00,0x00,0x00,0x78,0x78,0x03,0xAC,
                        0x21,0x00,0x04,0x00,0x00,0x78,0x78,0x3e,0x00,0x06,
                        0x00,0x00,0x00,0x00}, 44, 0},
    {0xA7, (uint8_t[]){0x19,0x19,0x00,0x64,0x40,0x07,0x16,0x40,0x00,0x04,
                        0x03,0x78,0x78,0x00,0x64,0x40,0x25,0x34,0x00,0x00,
                        0x02,0x01,0x78,0x78,0x00,0x64,0x40,0x4B,0x5A,0x00,
                        0x00,0x02,0x01,0x78,0x78,0x00,0x24,0x40,0x69,0x78,
                        0x00,0x00,0x00,0x00,0x78,0x78,0x00,0x44}, 48, 0},
    {0xAC, (uint8_t[]){0x08,0x0A,0x11,0x00,0x13,0x03,0x1B,0x18,0x06,0x1A,
                        0x19,0x1B,0x1B,0x1B,0x18,0x1B,0x09,0x0B,0x10,0x02,
                        0x12,0x01,0x1B,0x18,0x06,0x1A,0x19,0x1B,0x1B,0x1B,
                        0x18,0x1B,0xFF,0x67,0xFF,0x67,0x00}, 37, 0},
    {0xAD, (uint8_t[]){0xCC,0x40,0x46,0x11,0x04,0x78,0x78}, 7, 0},
    {0xE8, (uint8_t[]){0x30,0x07,0x00,0x94,0x94,0x9C,0x00,0xE2,0x04,0x00,
                        0x00,0x00,0x00,0xEF}, 14, 0},
    {0xE7, (uint8_t[]){0x8B,0x3C,0x00,0x0C,0xF0,0x5D,0x00,0x5D,0x00,0x5D,
                        0x00,0x5D,0x00,0xFF,0x00,0x08,0x7B,0x00,0x00,0xC8,
                        0x6A,0x5A,0x08,0x1A,0x3C,0x00,0x81,0x01,0xCC,0x01,
                        0x7F,0xF0,0x22}, 33, 0},
    // Sleep out
    {0x11, NULL, 0, 600},
    // Display on
    {0x29, NULL, 0, 120},
};
#endif /* CONFIG_LV_TFT_DISPLAY_CONTROLLER_ST7102 */

/* ------------------------------------------------------------------------ */
/* Driver state                                                              */
/* ------------------------------------------------------------------------ */

static esp_lcd_dsi_bus_handle_t dsi_bus = NULL;
static esp_lcd_panel_io_handle_t dbi_io = NULL;
static esp_lcd_panel_handle_t dpi_panel = NULL;
static esp_ldo_channel_handle_t ldo_phy_chan = NULL;

/* ------------------------------------------------------------------------ */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------ */

static void dsi_enable_phy_power(void)
{
    // VDD_MIPI_DPHY is normally supplied with 2.5V. When it is wired to an
    // internal LDO, acquire that channel here so the PHY can come out of
    // the "No Power" state.
    if (DSI_PHY_LDO_CHAN < 0) {
        return;
    }
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = DSI_PHY_LDO_CHAN,
        .voltage_mv = DSI_PHY_LDO_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_cfg, &ldo_phy_chan));
}

static void dsi_panel_reset(void)
{
#if DSI_PANEL_RST >= 0
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << DSI_PANEL_RST,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);
    gpio_set_level(DSI_PANEL_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(DSI_PANEL_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
#else
    (void)0;
#endif
}

static void dsi_send_init_sequence(const dsi_init_cmd_t *cmds, size_t cnt)
{
    for (size_t i = 0; i < cnt; i++) {
        // Log every command so that, if the DSI host ever hangs waiting for a
        // panel command-ACK, the serial log shows exactly which command was
        // being sent when it stopped.
        ESP_LOGI(TAG, "init cmd[%u]/%u: 0x%02X (%u param bytes, %u ms delay)",
                 i, (unsigned)cnt, cmds[i].cmd, (unsigned)cmds[i].len,
                 (unsigned)cmds[i].delay_ms);
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(dbi_io, cmds[i].cmd,
                                                  cmds[i].data, cmds[i].len));
        if (cmds[i].delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(cmds[i].delay_ms));
        }
    }
    ESP_LOGI(TAG, "Panel init sequence sent (%u commands)", (unsigned)cnt);
}

/* ------------------------------------------------------------------------ */
/* Public API                                                                */
/* ------------------------------------------------------------------------ */

void dsi_panel_init(void)
{
    ESP_LOGI(TAG, "Power on MIPI DSI PHY");
    dsi_enable_phy_power();

    // Reset the panel controller (if wired to a GPIO)
    dsi_panel_reset();

    // Create the MIPI-DSI bus (also initializes the DSI PHY)
    esp_lcd_dsi_bus_config_t bus_config = {
        .bus_id = 0,
        .num_data_lanes = DSI_LANE_NUM,
        .lane_bit_rate_mbps = DSI_LANE_BITRATE,
    };
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_config, &dsi_bus));

    // Command channel (DBI) used to initialize the panel controller
    esp_lcd_dbi_io_config_t dbi_config = {
        .virtual_channel = 0,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(dsi_bus, &dbi_config, &dbi_io));

    // Video mode (DPI) panel
    esp_lcd_dpi_panel_config_t dpi_config = {
        .virtual_channel = 0,
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = DPI_CLK_MHZ,
        .in_color_format = DSI_IN_COLOR_FMT,
        .out_color_format = DSI_OUT_COLOR_FMT,
        .num_fbs = 1,
        .video_timing = {
            .h_size = DSI_H_RES,
            .v_size = DSI_V_RES,
            .hsync_pulse_width = DSI_HSYNC,
            .hsync_back_porch = DSI_HBP,
            .hsync_front_porch = DSI_HFP,
            .vsync_pulse_width = DSI_VSYNC,
            .vsync_back_porch = DSI_VBP,
            .vsync_front_porch = DSI_VFP,
        },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_dpi(dsi_bus, &dpi_config, &dpi_panel));

    // Send the controller init sequence over the DBI command channel.
    // The controller and its register/video-timing settings are chosen from
    // menuconfig (ST7701 / ST7701S by default, ST7102 for OSPTEK 4.3" modules).
    ESP_LOGI(TAG, "Send panel init sequence");
#if defined(CONFIG_LV_TFT_DISPLAY_CONTROLLER_ST7102)
    dsi_send_init_sequence(st7102_init, sizeof(st7102_init) / sizeof(st7102_init[0]));
#else
    dsi_send_init_sequence(st7701s_init, sizeof(st7701s_init) / sizeof(st7701s_init[0]));
#endif

    // Start the DPI video stream (panel keeps refreshing from its frame buffer)
    ESP_ERROR_CHECK(esp_lcd_panel_init(dpi_panel));

    ESP_LOGI(TAG, "MIPI-DSI panel ready: %dx%d, %d lanes @ %dMbps, %dMHz pixel clock",
             DSI_H_RES, DSI_V_RES, DSI_LANE_NUM, DSI_LANE_BITRATE, DPI_CLK_MHZ);
}

void dsi_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area,
                       lv_color_t *color_map)
{
    // Copy the rendered area into the DPI panel frame buffer. The DPI driver
    // copies the buffer synchronously and the panel refreshes from its own
    // frame buffer in the background, so we can signal LVGL right away.
    esp_lcd_panel_draw_bitmap(dpi_panel, area->x1, area->y1,
                              area->x2 + 1, area->y2 + 1, color_map);
    lv_disp_flush_ready(drv);
}
