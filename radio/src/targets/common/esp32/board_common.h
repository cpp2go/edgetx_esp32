#pragma once  // 确保有这一行防止重复包含

// 必须显式包含 FreeRTOS 基础头文件
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// 针对 IDF 5.x 的兼容性宏定义
#ifndef RTOS_WAIT_MS
#define RTOS_WAIT_MS(ms) vTaskDelay(pdMS_TO_TICKS(ms))
#endif

#if defined(ESP_PLATFORM)
#include <rtos.h>
#else
// for YAML generation
#endif

#include <inttypes.h>
#include "definitions.h"
#include "edgetx_constants.h"
#include "hal.h"
#include "hal/serial_port.h"
#include "hal/watchdog_driver.h"

#define SYSTEM_TICKS_1MS pdMS_TO_TICKS(1)

static inline uint32_t ticksNow() {
#if defined(ESP_PLATFORM)
    return xTaskGetTickCount();
#else
    // for yaml_data
    return 0;
#endif
}

void init2MhzTimer();

