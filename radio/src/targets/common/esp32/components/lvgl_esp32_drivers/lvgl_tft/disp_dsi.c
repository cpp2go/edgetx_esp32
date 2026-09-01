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
 * The default controller is ST7701 / ST7701S. The init sequence and the video
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
    uint8_t data[16];         /* command parameters */
    uint8_t len;              /* number of parameters */
    uint32_t delay_ms;        /* delay after the command (0 = none) */
} dsi_init_cmd_t;

/* Default init sequence for a generic ST7701S 320x480 MIPI-DSI panel.
 * NOTE: register values are panel vendor specific, adjust them to the exact
 * module you are using (they control scan direction, porch, VCOM, gamma...).
 */
static const dsi_init_cmd_t st7701s_init[] = {
    // Enable manufacturer command access
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x10}, 5, 0},
    // Line setting
    {0xC0, {0x3B, 0x00}, 2, 0},
    // Porch setting
    {0xC1, {0x0D, 0x02}, 2, 0},
    // Porch setting
    {0xC2, {0x31, 0x05}, 2, 0},
    // Gate setting
    {0xC3, {0x01, 0x00}, 2, 0},
    // Source setting
    {0xC4, {0x31, 0x00}, 2, 0},
    // MIPI setting (lane count / speed, panel specific)
    {0xC5, {0x00, 0x02}, 2, 0},
    // VCOM setting
    {0xC6, {0x1D, 0x00}, 2, 0},
    // Vendor setting
    {0xC7, {0x10}, 1, 0},
    // Vendor setting
    {0xC8, {0x03, 0x00}, 2, 0},
    // Power setting
    {0xD0, {0x9D, 0x0D}, 2, 0},
    // VCOM setting
    {0xD1, {0x31, 0x15}, 2, 0},
    // Power setting
    {0xD2, {0x22, 0x02}, 2, 0},
    // Vendor setting
    {0xD3, {0x08, 0x00}, 2, 0},
    // Vendor setting
    {0xD4, {0x6D}, 1, 0},
    // Vendor setting
    {0xD5, {0x26, 0x06}, 2, 0},
    // Gamma positive
    {0xE0, {0x00, 0x20, 0x00}, 3, 0},
    // Gamma negative
    {0xE1, {0x00, 0x20, 0x00}, 3, 0},
    // Disable manufacturer command access
    {0xFF, {0x77, 0x01, 0x00, 0x00, 0x11}, 5, 0},
    // Sleep out
    {0x11, {0x00}, 0, 120},
    // Display on
    {0x29, {0x00}, 0, 20},
};

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
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(dbi_io, cmds[i].cmd,
                                                  cmds[i].data, cmds[i].len));
        if (cmds[i].delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(cmds[i].delay_ms));
        }
    }
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

    // Send the controller init sequence over the DBI command channel
    ESP_LOGI(TAG, "Send panel init sequence");
    dsi_send_init_sequence(st7701s_init, sizeof(st7701s_init) / sizeof(st7701s_init[0]));

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
