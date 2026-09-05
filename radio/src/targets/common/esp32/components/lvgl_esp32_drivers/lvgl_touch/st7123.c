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

#include <esp_log.h>
#include <esp_timer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include <lvgl.h>
#else
#include <lvgl/lvgl.h>
#endif
#include "st7123.h"
#include "lvgl_i2c/i2c_manager.h"

#define TAG "ST7123"

static i2c_master_dev_handle_t st7123_handle = NULL;
static bool st7123_inited = false;

/* Keep the last reported position so a release still carries coordinates. */
static lv_coord_t last_x = 0;
static lv_coord_t last_y = 0;

/* Native resolution read from the controller (defaults to 480x800). */
static uint16_t touch_native_x_max = 480;
static uint16_t touch_native_y_max = 800;
/* Max simultaneous touches reported by the controller (cached at init). */
static uint8_t touch_max_touches = 5;

/* Map native controller coordinates into the logical (LVGL) frame. The ST7123
 * reports in the panel's native orientation, so when the UI is rendered
 * rotated 90 deg (CONFIG_LV_TFT_DSI_UI_ROTATE_90) the same transform used by
 * the display flush must be undone here. */
static void st7123_map_coords(lv_coord_t *x, lv_coord_t *y)
{
    int64_t native_x = *x;
    int64_t native_y = *y;

#if defined(CONFIG_LV_TFT_DSI_UI_ROTATE_90)
    /* Display rotation used in disp_dsi.c (DSI_ROT_CW):
     *   logical(px_phys, py_phys) -> log_x = py_phys, log_y = maxX-1-px_phys
     * so undo it here (scale into LV_HOR_RES x LV_VER_RES). */
    int64_t lx = native_y;
    int64_t ly = (int64_t)(touch_native_x_max - 1) - native_x;
    *x = (lv_coord_t)(lx * LV_HOR_RES / touch_native_y_max);
    *y = (lv_coord_t)(ly * LV_VER_RES / touch_native_x_max);
#else
    *x = (lv_coord_t)(native_x * LV_HOR_RES / touch_native_x_max);
    *y = (lv_coord_t)(native_y * LV_VER_RES / touch_native_y_max);
#endif
}

static esp_err_t st7123_read_reg(uint16_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t addr[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    /* Bounded timeout: a non-responsive controller must never block the GUI
     * task that polls the touch driver (timeout -1 would wait forever). */
    return i2c_master_transmit_receive(st7123_handle, addr, sizeof(addr),
                                       buf, len, 50);
}

static void st7123_fill_data(lv_indev_data_t *data, bool pressed)
{
    data->point.x = last_x;
    data->point.y = last_y;
    data->state = pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

/**
  * @brief  Initialize communication with the ST7123 controller.
  * @param  dev_addr: I2C slave address (7-bit) of the ST7123.
  */
void st7123_init(uint16_t dev_addr)
{
    esp_err_t ret;

    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = 400000,
        .device_address = dev_addr,
    };
    ret = i2c_master_bus_add_device(lvgl_i2c_bus_handle, &i2c_dev_conf,
                                    &st7123_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c add device failed: %s", esp_err_to_name(ret));
        return;
    }

    /* Read back the panel resolution / max touches. */
    uint8_t info[5] = {0};  /* max_x_h, max_x_l, max_y_h, max_y_l, touches */
    ret = st7123_read_reg(ST7123_MAX_X_COORD_H_REG, info, sizeof(info));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to read touch info: %s", esp_err_to_name(ret));
        return;
    }
    touch_native_x_max = (uint16_t)(((info[0] & 0x3F) << 8) | info[1]);
    touch_native_y_max = (uint16_t)(((info[2] & 0x3F) << 8) | info[3]);
    if (touch_native_x_max == 0) touch_native_x_max = 480;
    if (touch_native_y_max == 0) touch_native_y_max = 800;
    touch_max_touches = info[4];
    if (touch_max_touches == 0 || touch_max_touches > ST7123_MAX_TOUCHES) {
        touch_max_touches = 5;
    }
    ESP_LOGI(TAG, "Found ST7123 touch panel (max X: %u, max Y: %u, touches: %u)",
             touch_native_x_max, touch_native_y_max, touch_max_touches);

    st7123_inited = true;
}

/**
  * @brief  Get the touch screen X and Y positions. Ignores multi touch.
  * @param  drv: LVGL indev driver (may be NULL).
  * @param  data: Store the data here.
  * @retval Always false.
  */
bool st7123_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    if (!st7123_inited || st7123_handle == NULL) {
        if (data) {
            st7123_fill_data(data, false);
        }
        return false;
    }

    /* Report protocol (official Espressif esp_lcd_touch_st7123):
     *   0x0010 = 1-byte advanced-info; bit3 (with_coord) is set when a fresh
     *   coordinate report is available. Only then read the report entries at
     *   0x0014 (REPORT_COORD_0_REG), 7 bytes per touch. */
    uint8_t adv_info = 0;
    if (st7123_read_reg(ST7123_ADV_INFO_REG, &adv_info, 1) != ESP_OK) {
        ESP_LOGE(TAG, "Error reading advanced info register");
        if (data) {
            st7123_fill_data(data, false);
        }
        return false;
    }

    /* Bring-up diagnostic: report the raw advanced-info byte whenever it
     * changes (press <-> release shows up as a bit toggle in the log). */
    {
        static uint8_t last_adv = 0xFF;
        if (adv_info != last_adv) {
            last_adv = adv_info;
            ESP_LOGI(TAG, "adv_info=0x%02X (with_coord=%d)", adv_info,
                     !!(adv_info & ST7123_ADV_INFO_WITH_COORD));
        }
    }

    if ((adv_info & ST7123_ADV_INFO_WITH_COORD) == 0) {
        /* No fresh report: panel is not touched. */
        if (data) {
            st7123_fill_data(data, false);
        }
        return false;
    }

    uint16_t num_touches = touch_max_touches;
    if (num_touches == 0 || num_touches > ST7123_MAX_TOUCHES) {
        num_touches = ST7123_MAX_TOUCHES;
    }

    uint8_t report[ST7123_MAX_TOUCHES * ST7123_TOUCH_REPORT_BYTES] = {0};
    if (st7123_read_reg(ST7123_REPORT_COORD_0_REG, report,
                        num_touches * ST7123_TOUCH_REPORT_BYTES) != ESP_OK) {
        ESP_LOGE(TAG, "Error reading touch report");
        if (data) {
            st7123_fill_data(data, false);
        }
        return false;
    }

    /* Report entry: | x_h:6 | rsv:1 | valid:1 |, x_l, y_h, y_l, area, ... */
    bool pressed = false;
    for (uint16_t i = 0; i < num_touches; i++) {
        const uint8_t *t = &report[i * ST7123_TOUCH_REPORT_BYTES];
        if (!(t[0] & 0x80)) {  /* valid flag */
            continue;
        }
        lv_coord_t x = (lv_coord_t)(((t[0] & 0x3F) << 8) | t[1]);
        lv_coord_t y = (lv_coord_t)(((t[2] & 0x3F) << 8) | t[3]);

        /* Scale / rotate native coords into the logical LVGL frame. */
        st7123_map_coords(&x, &y);

#if CONFIG_LV_ST7123_SWAPXY
        lv_coord_t swap = x;
        x = y;
        y = swap;
#endif
#if CONFIG_LV_ST7123_INVERT_X
        x = (lv_coord_t)(LV_HOR_RES - 1 - x);
#endif
#if CONFIG_LV_ST7123_INVERT_Y
        y = (lv_coord_t)(LV_VER_RES - 1 - y);
#endif

        last_x = x;
        last_y = y;
        pressed = true;
        break;  /* single-touch UI: only the first contact matters */
    }

    if (data) {
        st7123_fill_data(data, pressed);
    }
    return false;
}
