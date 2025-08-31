// ANCHOR: Overview
// SECTION - Includes ---------------------------------------------------------
// Logger task: periodically prints sensor readings
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "app_config.h"
#include "logging.h"
#include "sensors_bmp390.h"
#include "sensors_usfsmax.h"
// !SECTION
extern float heading[2];
extern float angle[2][2];

// SECTION - Task -------------------------------------------------------------
static void task_logger(void *param) {
  const TickType_t period = pdMS_TO_TICKS(LOGGER_PERIOD_MS);
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    bmp_reading_t r;
    if (bmp390_get(r) && r.valid) {
      LOGF("BMP1 T:%7.2fC P:%8.2fhPa Alt:%8.2fm ", r.temperature_c, r.pressure_pa / 100.0f, r.altitude_m);
    }
    usfs_reading_t u;
    if (usfsmax_get(u) && u.valid) {
      // Compute Euler from quaternion when debugging, else fallback to firmware Euler
#if DEBUG_ENABLED
      const float w = u.quat[0], x = u.quat[1], y = u.quat[2], z = u.quat[3];
      const float nq = (w*w + x*x + y*y + z*z);
      if (nq > 1e-6f) {
        float heading_deg = atan2f(2.0f*(x*y + w*z), 1.0f - 2.0f*(y*y + z*z)) * 57.2957795f;
        float pitch_deg   = asinf(2.0f*(w*y - z*x)) * 57.2957795f;
        float roll_deg    = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y)) * 57.2957795f;
        LOGF("USFS YPR:%8.2f,%8.2f,%8.2f ", heading_deg, pitch_deg, roll_deg);
      } else {
        LOGF("USFS YPR:%8.2f,%8.2f,%8.2f ", heading[0], angle[0][0], angle[0][1]);
      }
#endif
      LOGF("AXYZ(g):%7.3f,%7.3f,%7.3f ", u.accel_g[0], u.accel_g[1], u.accel_g[2]);
    }
    LOGLN("");
    vTaskDelayUntil(&last, period);
  }
}
// !SECTION

void logger_start_task() {
  xTaskCreatePinnedToCore(task_logger, "logger", TASK_STACK_LOGGER, nullptr, TASK_PRIO_LOGGER, nullptr, APP_CPU_NUM);
}
