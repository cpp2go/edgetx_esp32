/*
 * Copyright (C) EdgeTX
 *
 * Based on code named
 *   opentx - https://github.com/opentx/opentx
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "edgetx.h"
#include "esp32_uart_driver.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"
#define TAG "UART"

// Largest frame we keep a copy of for echo verification (CRSF channel frames
// are 26 bytes, the longest handset command is well below this).
#define ESP_UART_ECHO_MAX  64
// Bytes held back while an echo is being verified.
#define ESP_UART_PEND_MAX  160

typedef struct {
    uart_port_t port;
    void (*on_idle_cb)(void *);
    void *on_idle_cb_param;
    QueueHandle_t uart_queue;
    TaskHandle_t rx_task;
    bool single_wire;
    // Echo verification: the last transmitted frame (single-wire S.Port bay
    // receives its own transmissions back). Received bytes are only discarded
    // while they still match this frame byte for byte.
    uint8_t  echo_buf[ESP_UART_ECHO_MAX];
    uint16_t echo_len;
    uint16_t echo_pos;
    int64_t  echo_deadline;   // us; the echo must arrive before this
    uint32_t baudrate;
    // Bytes waiting to be handed to the protocol driver. While an echo is
    // being matched these also hold the tentatively dropped bytes, so they can
    // be restored if the frame turns out not to be an echo.
    uint8_t  pend_buf[ESP_UART_PEND_MAX];
    uint16_t pend_head;
    uint16_t pend_len;
    // Protects echo_* and pend_* shared between mixer TX and UART RX tasks.
    portMUX_TYPE echo_lock;
} esp32_uart_ctx_t;

static esp32_uart_ctx_t uarts[SOC_UART_HP_NUM]; // TODO-OPENX1 init

// Debug counters, see espUartGetRxStats()
static volatile uint32_t uartDbgDataEvents[SOC_UART_HP_NUM];
static volatile uint32_t uartDbgRxBytes[SOC_UART_HP_NUM];

void espUartGetRxStats(int port, uint32_t* events, uint32_t* bytes)
{
    if (port < 0 || port >= SOC_UART_HP_NUM) {
        if (events) *events = 0;
        if (bytes) *bytes = 0;
        return;
    }
    if (events) *events = uartDbgDataEvents[port];
    if (bytes) *bytes = uartDbgRxBytes[port];
}

void espUartResetRxStats()
{
    for (int i = 0; i < SOC_UART_HP_NUM; i += 1) {
        uartDbgDataEvents[i] = 0;
        uartDbgRxBytes[i] = 0;
    }
}

static inline esp32_uart_ctx_t *ctx_to_port(void* ctx) {
    return (esp32_uart_ctx_t *)ctx;
}

static void espuart_setIdleCb(void* ctx, void (*on_idle)(void*), void* param) {
    esp32_uart_ctx_t *port = ctx_to_port(ctx);
    port->on_idle_cb = on_idle;
    port->on_idle_cb_param = param;
}

static void espuart_uart_event_task(void *pvParameters)
{
    esp32_uart_ctx_t *port = ctx_to_port(pvParameters);
    uart_event_t event;
    size_t buffered_size;
    while (true) {
        //Waiting for UART event.
        if (xQueueReceive(port->uart_queue, (void *)&event, (TickType_t)portMAX_DELAY)) {
            switch (event.type) {
            //Event of UART receving data
            /*We'd better handler data event fast, there would be much more data events than
            other types of events. If we take too much time on data event, the queue might
            be full.*/
            case UART_DATA:
                //ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                uartDbgDataEvents[port->port] += 1;
                uartDbgRxBytes[port->port] += (uint32_t)event.size;
                if (NULL != port->on_idle_cb) {
                    port->on_idle_cb(port->on_idle_cb_param);
                }
                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGW(TAG, "hw fifo overflow");
                uart_flush_input(port->port);
                xQueueReset(port->uart_queue);
                taskENTER_CRITICAL(&port->echo_lock);
                port->pend_head = port->pend_len = 0;
                port->echo_pos = port->echo_len = 0;
                taskEXIT_CRITICAL(&port->echo_lock);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGW(TAG, "ring buffer full");
                uart_flush_input(port->port);
                xQueueReset(port->uart_queue);
                taskENTER_CRITICAL(&port->echo_lock);
                port->pend_head = port->pend_len = 0;
                port->echo_pos = port->echo_len = 0;
                taskEXIT_CRITICAL(&port->echo_lock);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                //ESP_LOGI(TAG, "uart rx break");
                if (NULL != port->on_idle_cb) {
                    port->on_idle_cb(port->on_idle_cb_param);
                }
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                break;
            //UART_PATTERN_DET
            case UART_PATTERN_DET:
            {
                uart_get_buffered_data_len(port->port, &buffered_size);
                int pos = uart_pattern_pop_pos(port->port);
                ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                break;
            }
            //Others
            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
        }
    }
}

static void get_serial_config(const etx_serial_init* params, uart_config_t *uart_config) {
    switch (params->encoding) {
    case ETX_Encoding_8N1:
        uart_config->parity = UART_PARITY_DISABLE;
        uart_config->stop_bits = UART_STOP_BITS_1;
        break;
    case ETX_Encoding_8E2:
        uart_config->parity = UART_PARITY_EVEN;
        uart_config->stop_bits = UART_STOP_BITS_2;
        break;
    default:
        break;
    }
}

void* espUartSerialStart(void *hw_def, const etx_serial_init* params)
{
    etx_esp32_uart_hw_def_t *hw = (etx_esp32_uart_hw_def_t *)hw_def;
    esp32_uart_ctx_t *port = &uarts[(int)hw->uart_port];
    portMUX_INITIALIZE(&port->echo_lock);
    port->port = hw->uart_port;
    port->pend_head = port->pend_len = 0;
    port->echo_len = port->echo_pos = 0;
    port->echo_deadline = 0;
    port->single_wire = hw->single_wire ||
                        (hw->single_wire_cb && hw->single_wire_cb());
    port->baudrate = params->baudrate ? params->baudrate : 115200;

    uart_config_t uart_config = {
        .baud_rate = (int)params->baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    get_serial_config(params, &uart_config);

    // We won't use a buffer for sending data.
    uart_driver_install(port->port, hw->fifoSize, 0,
        hw->queueSize, &port->uart_queue,
        ESP_INTR_FLAG_IRAM);
    uart_param_config(port->port, &uart_config);
    uart_set_pin(port->port, hw->tx_pin, hw->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // The event task runs the telemetry frame polling synchronously (see
    // telemetryFrameTrigger_ISR), so it must not share the UI core: a
    // fullscreen Lua widget keeps the menus task (CPU1) busy for >150ms per
    // frame and would starve polling until the RX ring overflows ("no
    // telemetry" with a perfectly good link). The mixer core sleeps most of
    // the time, so pin the task there, at a priority that stays below the
    // mixer/menus tasks so it never preempts them.
    xTaskCreatePinnedToCore(espuart_uart_event_task, "espuart_uart_event_task",
        4096, (void *)port, 2, &port->rx_task,
#if defined(MIXER_TASK_CORE)
        MIXER_TASK_CORE
#else
        tskNO_AFFINITY
#endif
    );
    return (void *)port;
}

void espUartSendByte(void* ctx, uint8_t byte)
{
    esp32_uart_ctx_t *port = ctx_to_port(ctx);
    uint8_t data = byte;
    uart_write_bytes(port->port, &data, 1);
}

// Remember what was just transmitted so the echo can be recognised (single-wire
// S.Port mode only). Only buffer-sent frames are tracked.
static void espUartArmEcho(esp32_uart_ctx_t *port, const uint8_t *data, uint32_t size)
{
    if (size == 0 || size > ESP_UART_ECHO_MAX) {
        taskENTER_CRITICAL(&port->echo_lock);
        port->echo_len = port->echo_pos = 0;
        taskEXIT_CRITICAL(&port->echo_lock);
        return;
    }
    int64_t tx_us = (int64_t)size * 10 * 1000000 / port->baudrate;
    int64_t deadline = esp_timer_get_time() + tx_us + 3000;
    taskENTER_CRITICAL(&port->echo_lock);
    memcpy(port->echo_buf, data, size);
    port->echo_len = (uint16_t)size;
    port->echo_pos = 0;
    port->echo_deadline = deadline;
    taskEXIT_CRITICAL(&port->echo_lock);
}

void espUartSendBuffer(void* ctx, const uint8_t * data, uint32_t size)
{
    esp32_uart_ctx_t *port = ctx_to_port(ctx);
    if (port->single_wire) {
        espUartArmEcho(port, data, size);
    }
    uart_write_bytes(port->port, data, size);
}

// Single-wire S.Port mode: discard received bytes while they still match the
// frame we just sent. Caller must hold port->echo_lock.
static inline bool espUartRxIsEchoLocked(esp32_uart_ctx_t *port, uint8_t byte, int64_t now)
{
    if (port->echo_len == 0 || port->echo_pos >= port->echo_len) return false;

    if (now > port->echo_deadline) {
        // Too late to be our own transmission -> release what was held.
        port->echo_pos = port->echo_len = 0;
        return false;
    }

    if (byte == port->echo_buf[port->echo_pos]) {
        if (port->pend_len >= ESP_UART_PEND_MAX) {
            // Cannot hold any more: keep the data instead of losing it.
            port->echo_pos = 0;
            return false;
        }
        // Hold the match back instead of dropping it right away: if the frame
        // turns out not to be an echo these bytes are real telemetry.
        port->pend_buf[port->pend_len++] = byte;
        if (++port->echo_pos == port->echo_len) {
            // Complete echo: drop exactly the bytes it added, older telemetry
            // must stay queued.
            if (port->pend_len >= port->echo_len) {
                port->pend_len -= port->echo_len;
            } else {
                port->pend_len = 0;
            }
            if (port->pend_head > port->pend_len) port->pend_head = port->pend_len;
            port->echo_pos = port->echo_len = 0;
        }
        return true;
    }

    if (port->echo_pos == 0) {
        // Telemetry byte arrived ahead of our echo: pass it through without
        // disarming echo_len so the upcoming echo is still filtered.
        return false;
    }

    // Mid-frame mismatch -> tentatively held bytes are genuine data.
    // Re-sync if this byte starts the expected echo.
    port->echo_pos = 0;
    if (byte == port->echo_buf[0] && port->pend_len < ESP_UART_PEND_MAX) {
        port->pend_buf[port->pend_len++] = byte;
        port->echo_pos = 1;
        if (port->echo_pos == port->echo_len) {
            if (port->pend_len >= port->echo_len) {
                port->pend_len -= port->echo_len;
            } else {
                port->pend_len = 0;
            }
            if (port->pend_head > port->pend_len) port->pend_head = port->pend_len;
            port->echo_pos = port->echo_len = 0;
        }
        return true;
    }
    return false;
}

static int espUartRxRead(esp32_uart_ctx_t *port, uint8_t *out, uint32_t maxlen)
{
    // Standard two-wire bay: original direct read path with zero overhead.
    if (!port->single_wire) {
        int r = uart_read_bytes(port->port, out, maxlen, 0);
        return r > 0 ? r : 0;
    }

    uint32_t n = 0;

    // Drain already-confirmed bytes first. Bytes in pend_buf are confirmed
    // whenever echo_pos == 0 (nothing tentatively held at the tail).
    taskENTER_CRITICAL(&port->echo_lock);
    if (port->echo_pos == 0) {
        while (n < maxlen && port->pend_head < port->pend_len) {
            out[n++] = port->pend_buf[port->pend_head++];
        }
        if (port->pend_head >= port->pend_len) port->pend_head = port->pend_len = 0;
    }
    taskEXIT_CRITICAL(&port->echo_lock);

    // Batched read into a small stack buffer outside the critical section,
    // then run the echo state machine under echo_lock.
    while (n < maxlen) {
        uint8_t tmp[64];
        uint32_t room;
        taskENTER_CRITICAL(&port->echo_lock);
        room = (port->pend_len < ESP_UART_PEND_MAX) ? (ESP_UART_PEND_MAX - port->pend_len) : 0;
        taskEXIT_CRITICAL(&port->echo_lock);
        if (room == 0) break;
        if (room > sizeof(tmp)) room = sizeof(tmp);

        int r = uart_read_bytes(port->port, tmp, room, 0);
        if (r <= 0) break;

        int64_t now = esp_timer_get_time();
        taskENTER_CRITICAL(&port->echo_lock);
        for (int i = 0; i < r; i++) {
            if (espUartRxIsEchoLocked(port, tmp[i], now)) continue;
            if (port->pend_len < ESP_UART_PEND_MAX) {
                port->pend_buf[port->pend_len++] = tmp[i];
            }
        }
        if (port->echo_len && now > port->echo_deadline) {
            port->echo_pos = port->echo_len = 0;
        }
        if (port->echo_pos == 0) {
            while (n < maxlen && port->pend_head < port->pend_len) {
                out[n++] = port->pend_buf[port->pend_head++];
            }
            if (port->pend_head >= port->pend_len) port->pend_head = port->pend_len = 0;
        }
        taskEXIT_CRITICAL(&port->echo_lock);
    }

    // If an earlier partial echo timed out without any new bytes arriving,
    // release the held bytes now.
    if (n < maxlen) {
        int64_t now = esp_timer_get_time();
        taskENTER_CRITICAL(&port->echo_lock);
        if (port->echo_len && now > port->echo_deadline) {
            port->echo_pos = port->echo_len = 0;
        }
        if (port->echo_pos == 0) {
            while (n < maxlen && port->pend_head < port->pend_len) {
                out[n++] = port->pend_buf[port->pend_head++];
            }
            if (port->pend_head >= port->pend_len) port->pend_head = port->pend_len = 0;
        }
        taskEXIT_CRITICAL(&port->echo_lock);
    }
    return (int)n;
}

void espUartWaitForTxCompleted(void* ctx)
{
    esp32_uart_ctx_t *port = ctx_to_port(ctx);
    uart_wait_tx_done(port->port, portMAX_DELAY);
}

static bool espUartTxCompleted(void* ctx)
{
    esp32_uart_ctx_t *port = ctx_to_port(ctx);
    return uart_wait_tx_done(port->port, 0) == ESP_OK;
}

static int espUartGetByte(void* ctx, uint8_t* data)
{
    esp32_uart_ctx_t *port = ctx_to_port(ctx);
    return espUartRxRead(port, data, 1);
}

static void espUartClearRxBuffer(void* ctx)
{
    esp32_uart_ctx_t *port = ctx_to_port(ctx);
    uart_flush(port->port);
    if (port->single_wire) {
        taskENTER_CRITICAL(&port->echo_lock);
        port->pend_head = port->pend_len = 0;
        port->echo_pos = port->echo_len = 0;
        taskEXIT_CRITICAL(&port->echo_lock);
    }
}

static void espUartSerialStop(void* ctx)
{
    esp32_uart_ctx_t *port = ctx_to_port(ctx);
    // Drop the callback first: the port struct is reused by the next protocol
    // that gets installed on this UART (e.g. CRSF -> MULTI), and a stale
    // callback would push frames to the wrong protocol driver.
    port->on_idle_cb = NULL;
    port->on_idle_cb_param = NULL;
    if (port->rx_task) {
        vTaskDelete(port->rx_task);
        port->rx_task = NULL;
    }
    uart_driver_delete(port->port);
    port->pend_head = port->pend_len = 0;
    port->echo_pos = port->echo_len = 0;
}

int espUartGetBufferedBytes(void* ctx) {
    esp32_uart_ctx_t *port = ctx_to_port(ctx);
    size_t size = 0U;
    if (ESP_OK != uart_get_buffered_data_len(port->port, &size)) {
      size = 0U;
    }
    // Bytes held back for echo verification are not available yet.
    return (int)size + (port->echo_len ? 0 : (int)(port->pend_len - port->pend_head));
}

int espUartCopyRxBuffer(void* ctx, uint8_t* buf, uint32_t len) {
    esp32_uart_ctx_t *port = ctx_to_port(ctx);
    int r = espUartRxRead(port, buf, len);
#if defined(OPENX1_UART_DUMP)
    // Diagnostic for missing telemetry: dump what actually arrives on the UART
    // so a valid CRSF stream can be told apart from noise / a wrong baudrate.
    // Rate limited to one sample per port per second.
    if (r > 0) {
        static TickType_t lastDump[UART_NUM_MAX];
        TickType_t now = xTaskGetTickCount();
        if (port->port < UART_NUM_MAX &&
            (now - lastDump[port->port]) >= pdMS_TO_TICKS(1000)) {
            lastDump[port->port] = now;
            char hex[3 * 24 + 1];
            int n = r > 24 ? 24 : r;
            for (int i = 0; i < n; i++) {
                snprintf(hex + i * 3, 4, "%02x ", buf[i]);
            }
            hex[n * 3] = '\0';
            ESP_LOGI(TAG, "uart%d rx %d bytes: %s", (int)port->port, r, hex);
        }
    }
#endif
    return r;
}

const etx_serial_driver_t ESPUartSerialDriver = {
    .init = espUartSerialStart,
    .deinit = espUartSerialStop,
    .sendByte = espUartSendByte,
    .sendBuffer = espUartSendBuffer,
    .txCompleted = espUartTxCompleted,
    .waitForTxCompleted = espUartWaitForTxCompleted,
    .getByte = espUartGetByte,
    .getBufferedBytes = espUartGetBufferedBytes,
    .copyRxBuffer = espUartCopyRxBuffer,
    .clearRxBuffer = espUartClearRxBuffer,
    .getBaudrate = nullptr,
    .setReceiveCb = nullptr,
    .setIdleCb = espuart_setIdleCb,
    .setBaudrateCb = nullptr,
};
