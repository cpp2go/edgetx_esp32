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

#pragma once

#ifdef __cplusplus
extern "C++" {
#endif

#if defined(SIMU)

  static inline void RTOS_START() {}

#elif defined(FREE_RTOS)

#if defined(ESP_PLATFORM)
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "freertos/semphr.h"
  #ifndef tskNO_AFFINITY
    #define tskNO_AFFINITY ((BaseType_t)0x7FFFFFFF)
  #endif
  #include "freertos/idf_additions.h"
#else
  #include <FreeRTOS/include/FreeRTOS.h>
  #include <FreeRTOS/include/task.h>
#endif

  typedef TaskHandle_t RTOS_TASK_HANDLE;

  #define RTOS_DEFINE_STACK(handle, stack_name, size) \
    static StackType_t stack_name[(size)]; \
    static StaticTask_t _##handle##_tcb

  #define RTOS_CREATE_TASK_EX(handle, func, name, stack, size, prio, core) \
    (handle) = xTaskCreateStaticPinnedToCore((func), (name), (size), NULL, (prio), (stack), &_##handle##_tcb, (core))

  #define RTOS_WAIT_MS(ms) vTaskDelay(pdMS_TO_TICKS(ms))
  #define RTOS_GET_MS()    (pdTICKS_TO_MS(xTaskGetTickCount()))

  static inline void RTOS_START()
  {
#ifdef ESP_PLATFORM
    // ESP-IDF starts the scheduler before EdgeTX; delete this task to free resources
    vTaskDelete(NULL);
#else
    vTaskStartScheduler();
#endif
  }

#endif  // RTOS type

#ifdef __cplusplus
}
#endif
