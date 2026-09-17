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

#include "mixer_scheduler.h"
#include "tasks/mixer_task.h"
#include "hal/usb_driver.h"
#include "os/sleep.h"

#include "dataconstants.h"
#include <string.h>

bool mixerSchedulerWaitForTrigger(uint8_t timeoutMs)
{
#if !defined(SIMU)
  uint32_t ulNotificationValue;
  const TickType_t xMaxBlockTime = pdMS_TO_TICKS( timeoutMs );

  /* Wait to be notified that the transmission is complete.  Note
     the first parameter is pdTRUE, which has the effect of clearing
     the task's notification value back to 0, making the notification
     value act like a binary (rather than a counting) semaphore.  */
  ulNotificationValue = ulTaskNotifyTake( pdTRUE, xMaxBlockTime );

  if( ulNotificationValue == 1 ) {
    /* The transmission ended as expected. */
    return false;

  } else {
    /* The call to ulTaskNotifyTake() timed out. */
    return true;
  }
#else
  // The simulator has nothing that raises a module interrupt, so without this the
  // caller would always wait out its full timeout and the mixer would end up running
  // at MIXER_MAX_PERIOD - 20 times a second. That is the pace at which the sticks are
  // then sampled (getADC() is called from the mixer) and at which channel frames
  // reach the module, so it is several tens of milliseconds of latency the radio
  // itself is adding. Emulate the trigger from the period the module driver asked for
  // (4 ms for CRSF at 400 kbaud) instead, which is what the hardware timer does.
  static uint32_t sinceTriggerUs = 0;
  const uint32_t periodUs = getMixerSchedulerPeriod();

  sinceTriggerUs += static_cast<uint32_t>(timeoutMs) * 1000u;
  sleep_ms(timeoutMs);

  if (sinceTriggerUs >= periodUs) {
    // Keep the remainder so the average rate stays right, unless we are so far
    // behind that catching up would only make it burst.
    sinceTriggerUs = (sinceTriggerUs < periodUs * 4u) ? sinceTriggerUs - periodUs : 0;
    return false;   // triggered: the mixer task runs a frame
  }

  return true;      // not yet: keep waiting
#endif
}

#if !defined(SIMU)

// Global trigger flag
#endif

// Mixer schedule
struct MixerSchedule {

  // period in us
  volatile uint16_t period;
};

static MixerSchedule mixerSchedules[NUM_MODULES];

uint16_t getMixerSchedulerPeriod()
{
#if defined(HARDWARE_INTERNAL_MODULE)
  if (mixerSchedules[INTERNAL_MODULE].period) {
    return mixerSchedules[INTERNAL_MODULE].period;
  }
#endif
#if defined(HARDWARE_EXTERNAL_MODULE)
  if (mixerSchedules[EXTERNAL_MODULE].period) {
    return mixerSchedules[EXTERNAL_MODULE].period;
  }
#endif
#if defined(STM32) && !defined(SIMU)
  if (getSelectedUsbMode() == USB_JOYSTICK_MODE) {
    return MIXER_SCHEDULER_JOYSTICK_PERIOD_US;
  }
#endif
  return MIXER_SCHEDULER_DEFAULT_PERIOD_US;
}

void mixerSchedulerInit()
{
  memset(mixerSchedules, 0, sizeof(mixerSchedules));
}

void mixerSchedulerSetPeriod(uint8_t moduleIdx, uint16_t periodUs)
{
  if (periodUs > 0 && periodUs < MIN_REFRESH_RATE) {
    periodUs = MIN_REFRESH_RATE;
  }
  else if (periodUs > 0 && periodUs > MAX_REFRESH_RATE) {
    periodUs = MAX_REFRESH_RATE;
  }

  mixerSchedules[moduleIdx].period = periodUs;
}

uint16_t mixerSchedulerGetPeriod(uint8_t moduleIdx)
{
  return mixerSchedules[moduleIdx].period;
}

#if !defined(SIMU)
void mixerSchedulerISRTrigger()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  /* At this point xTaskToNotify should not be NULL as
     a transmission was in progress. */
  configASSERT( mixerTaskId._rtos_handle != NULL );

  /* Notify the task that the transmission is complete. */
  vTaskNotifyGiveFromISR( mixerTaskId._rtos_handle,
                          &xHigherPriorityTaskWoken );

  /* If xHigherPriorityTaskWoken is now set to pdTRUE then a
     context switch should be performed to ensure the interrupt
     returns directly to the highest priority task.  The macro used
     for this purpose is dependent on the port in use and may be
     called portEND_SWITCHING_ISR(). */
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

#endif
