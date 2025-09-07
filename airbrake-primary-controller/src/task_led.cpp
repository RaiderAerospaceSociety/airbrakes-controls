// ===== LED Task =====
// Brief: Status LED — red on start, orange while waiting for boot readiness,
// green when fused values are ready, flashing yellow on boot faults.
//* -- Includes --
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <UMS3.h>

#include "app_config.h"
#include "board.h"
#include "services/fc.h"
#include "services/fusion.h"

static void task_led(void *param) {
  // Start: solid red
  ums3.setPixelColor(0xFF0000);

  bool blink_on = false;
  uint32_t last_blink_ms = 0;
  const uint32_t blink_period_ms = 400; // ~2.5 Hz

  for (;;) {
    // Gather status snapshots
    svc::FcStatus st; bool have_fc = svc::fcGetStatus(st);
    svc::FusedAlt f;  bool have_fused = svc::fusionGetAlt(f);

    // Determine phases/faults
    bool sensors_ok = false;
    bool agl_ready = false;
    bool fault = false;

    if (have_fc) {
      uint32_t ff = st.flags;
      // Consider IMU1 and BMP1 as required for boot; IMU2 optional
      sensors_ok = (ff & svc::FCF_SENS_IMU1_OK) && (ff & svc::FCF_SENS_BMP1_OK);
      fault = !sensors_ok; // treat missing required sensors as boot fault
    }
    if (have_fused) {
      agl_ready = f.agl_ready;
    }

    uint32_t now = millis();
    if ((now - last_blink_ms) >= blink_period_ms) { blink_on = !blink_on; last_blink_ms = now; }

    // Decide LED color
    uint32_t color = 0xFF0000; // default red (startup or no data yet)
    if (fault) {
      // Flashing yellow on boot fault
      color = blink_on ? 0xFFFF00 : 0x000000;
    } else if (sensors_ok && !agl_ready) {
      // Devices OK, waiting for baseline/AGL readiness -> orange
      color = 0xFFA500;
    } else if (sensors_ok && agl_ready) {
      // Fully ready -> green
      color = 0x00FF00;
    }

    ums3.setPixelColor(color);
    vTaskDelay(pdMS_TO_TICKS(LED_PERIOD_MS));
  }
}

//* -- API --
void ledStartTask() {
  xTaskCreatePinnedToCore(task_led, "led", TASK_STACK_LED, nullptr, TASK_PRIO_LED, nullptr, APP_CPU_NUM);
}
