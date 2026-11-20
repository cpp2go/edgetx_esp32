// simple MCU interface driver for the displays.
// Mostly from ESP examples/peripherals/lcd

static const char *TAG = "lcd-mcu";

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "lvgl.h"
#ifdef CONFIG_SPIRAM
#include "esp32s3/rom/cache.h"  // Cache_WriteBack_Addr: flush CPU D-cache to PSRAM before GDMA reads
#endif

#include "lvgl_helpers.h"
#include "disp_mcu.h"

// I80 parallel pixel clock. 26 MHz is the value this board is known to run
// cleanly; 40 MHz was tried and made the panel misbehave, 32 MHz is the next
// step worth testing before going higher.
#define MCU_LCD_PIXEL_CLOCK_HZ (26 * 1000 * 1000)

static esp_lcd_i80_bus_handle_t i80_bus = NULL;
static esp_lcd_i80_bus_config_t bus_config = {
    .dc_gpio_num = CONFIG_LV_TFT_MCU_DC_NUM,
    .wr_gpio_num = CONFIG_LV_TFT_MCU_WR_NUM,
    .clk_src = LCD_CLK_SRC_DEFAULT,
    .data_gpio_nums = {
        CONFIG_LV_TFT_MCU_DATA0,
        CONFIG_LV_TFT_MCU_DATA1,
        CONFIG_LV_TFT_MCU_DATA2,
        CONFIG_LV_TFT_MCU_DATA3,
        CONFIG_LV_TFT_MCU_DATA4,
        CONFIG_LV_TFT_MCU_DATA5,
        CONFIG_LV_TFT_MCU_DATA6,
        CONFIG_LV_TFT_MCU_DATA7,
#if CONFIG_LV_TFT_MCU_BUS_WIDTH_16
        CONFIG_LV_TFT_MCU_DATA8,
        CONFIG_LV_TFT_MCU_DATA9,
        CONFIG_LV_TFT_MCU_DATA10,
        CONFIG_LV_TFT_MCU_DATA11,
        CONFIG_LV_TFT_MCU_DATA12,
        CONFIG_LV_TFT_MCU_DATA13,
        CONFIG_LV_TFT_MCU_DATA14,
        CONFIG_LV_TFT_MCU_DATA15,
#endif
    },
#if CONFIG_LV_TFT_MCU_BUS_WIDTH_16
    .bus_width = 16,
#else
    .bus_width = 8,
#endif
    // Twice the largest stripe: the I80 driver sizes its DMA descriptor ring
    // from this, and a ring that only just fits one stripe cannot mount the
    // next transfer while the previous one is still finishing
    // ("gdma_link_mount_buffers: lli full").
    .max_transfer_bytes = DISP_BUF_SIZE * sizeof(lv_color_t) * 2
};

static bool mcu_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);

static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_panel_io_i80_config_t io_config = {
    .cs_gpio_num = CONFIG_LV_TFT_MCU_CS_NUM,
    .pclk_hz = MCU_LCD_PIXEL_CLOCK_HZ,
    .trans_queue_depth = 15,
    .dc_levels = {
        .dc_idle_level = 0,
        .dc_cmd_level = 0,
        .dc_dummy_level = 0,
        .dc_data_level = 1,
    },
    .flags = {
        .cs_active_high = 0,
        .reverse_color_bits = 0,
        .swap_color_bytes = 1,
        .pclk_active_neg = 0,
        .pclk_idle_low = 0,
    },
    .on_color_trans_done = mcu_notify_lvgl_flush_ready,
    .user_ctx = NULL,
#if CONFIG_LV_TFT_LCD_CMD_WIDTH_16
    .lcd_cmd_bits = 16,
#else
    .lcd_cmd_bits = 8,
#endif
#if CONFIG_LV_TFT_LCD_PARAM_WIDTH_16
    .lcd_param_bits = 16,
#else
    .lcd_param_bits = 8,
#endif
};

static esp_lcd_panel_handle_t panel_handle = NULL;
static esp_lcd_panel_dev_config_t panel_config = {
    .reset_gpio_num = CONFIG_LV_TFT_MCU_RST_NUM,
    //.color_space = ESP_LCD_COLOR_SPACE_BGR,
    .bits_per_pixel = 16,
};


// Signalled by the LCD DMA completion ISR, one count per finished color
// transaction. The panel driver copies the pixels into a single shared DMA
// buffer, so only one transaction may be in flight at a time and every stripe
// has to be waited for from task context before the next one is queued.
static SemaphoreHandle_t color_trans_done = NULL;

#define COLOR_TRANS_DONE_TMO_MS 100

static bool mcu_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    // Interrupt context: only signal. Queuing the next stripe from here would
    // call esp_lcd_panel_draw_bitmap() -> esp_lcd_panel_io_tx_param(), which
    // drains the done queue and busy-waits on the bus (not ISR safe).
    BaseType_t woken = pdFALSE;
    if (color_trans_done) {
        xSemaphoreGiveFromISR(color_trans_done, &woken);
    }
    return woken == pdTRUE;
}

// A lost completion must not freeze the UI task (the menus task owns the LVGL
// refresh): bail out and let the frame be torn instead of deadlocking.
static bool wait_color_trans_done(void)
{
    if (!color_trans_done) return false;
    return xSemaphoreTake(color_trans_done, pdMS_TO_TICKS(COLOR_TRANS_DONE_TMO_MS)) == pdTRUE;
}

void lcd_mcu_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    // x2 / y2 are exclusive here: EdgeTX passes rect_t::right() / bottom().
    const int width = area->x2 - area->x1;
    const int total_h = area->y2 - area->y1;
    if (width <= 0 || total_h <= 0) {
        lv_disp_flush_ready(drv);
        return;
    }

    int max_rows = DISP_BUF_SIZE / width;
    if (max_rows < 1) {
        max_rows = 1;
    }

    // LVGL renders through the CPU D-cache, but the pixels are read back by
    // the CPU (memcpy into dmabuf), which is cache coherent, so no explicit
    // write-back of the frame buffer is needed here.
    // Drop stale counts left over by a timed out stripe.
    while (color_trans_done && xSemaphoreTake(color_trans_done, 0) == pdTRUE) {}

    lv_color_t *pcolor = color_map;
    for (int y = area->y1; y < area->y2;) {
        int h = area->y2 - y;
        if (h > max_rows) h = max_rows;

#ifdef CONFIG_SPIRAM
        // The GDMA reads the frame buffer straight out of PSRAM below.
        Cache_WriteBack_Addr((uint32_t)pcolor,
                             (uint32_t)(width * h * sizeof(lv_color_t)));
#endif
        esp_lcd_panel_draw_bitmap(panel_handle, area->x1, y, area->x2, y + h, pcolor);
        y += h;
        pcolor += width * h;

        if (!wait_color_trans_done()) {
            break;
        }
    }

    // All pixels are on the wire: safe to hand the buffer back to LVGL.
    lv_disp_flush_ready(drv);
}

#if defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_HX8357
extern esp_err_t esp_lcd_new_panel_hx8357(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ILI9488
extern esp_err_t esp_lcd_new_panel_ili9488(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel);
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ST7796S
extern esp_err_t esp_lcd_new_panel_st7796s(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel);
#endif

// The panel GRAM powers up with random content and the backlight is enabled
// right after this init, so wipe it first: otherwise the radio shows a flash
// of noise until LVGL pushes its first frame.
static void panel_clear_gram(void)
{
    enum { CLEAR_ROWS = 16 };
    const int width = LV_HOR_RES_MAX;
    const int height = LV_VER_RES_MAX;
    const size_t bytes = (size_t)width * CLEAR_ROWS * sizeof(lv_color_t);

    lv_color_t *black = heap_caps_calloc(1, bytes, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!black) {
        ESP_LOGW(TAG, "no memory to clear panel GRAM");
        return;
    }

    for (int y = 0; y < height; y += CLEAR_ROWS) {
        int h = height - y;
        if (h > CLEAR_ROWS) h = CLEAR_ROWS;
        esp_lcd_panel_draw_bitmap(panel_handle, 0, y, width, y + h, black);
        if (!wait_color_trans_done()) {
            break;
        }
    }

    heap_caps_free(black);
}

void disp_mcu_panel_init(void) {
    ESP_LOGI(TAG, "Initialize Intel 8080 bus");

    if (!color_trans_done) {
        color_trans_done = xSemaphoreCreateBinary();
        ESP_ERROR_CHECK(color_trans_done == NULL ? ESP_ERR_NO_MEM : ESP_OK);
    }

    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));
#if defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_HX8357
    ESP_ERROR_CHECK(esp_lcd_new_panel_hx8357(io_handle, &panel_config, &panel_handle));
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ILI9488
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9488(io_handle, &panel_config, &panel_handle));
#elif defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_ST7796S
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7796s(io_handle, &panel_config, &panel_handle));
#endif
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    // the gap is LCD panel specific, even panels with the same driver IC, can have different gap value
    //esp_lcd_panel_set_gap(panel_handle, 0, 20);

    panel_clear_gram();
}
