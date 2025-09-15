// Airbrake Flight Controller (FSM) implementation
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "services/fc.h"
#include "services/fc_core.h"
#include "telemetry.h"
#include "config/fc_config.h"
#include "app_config.h"

namespace svc
{

  static SemaphoreHandle_t s_mutex = nullptr;
  static FcStatus s_stat = {};
  static FcCoreCtx s_ctx;    // core context (persistent)
  static bool s_core_inited = false;

  // Helpers
  // No more local flag/FSM helpers; logic resides in fc_core

  static void fc_task(void *param)
  {
    TickType_t last = xTaskGetTickCount();
    uint32_t prev_ms = millis();
    if (!s_mutex)
      s_mutex = xSemaphoreCreateMutex();
    if (!s_core_inited) { fc_init(s_ctx); s_core_inited = true; }
    for (;;)
    {
      TelemetryRecord rec;
      telemetryGetLatest(rec);
      const TelemetryFused &f = rec.fused;
      uint32_t now = millis();
      uint32_t dt = now - prev_ms;
      if (dt > 1000)
        dt = 1000;
      if (dt < 1)
        dt = 1;
      prev_ms = now;
      // Build core inputs from Telemetry snapshot
      FcInputs in{};
      in.dt_ms = dt;
      in.now_ms = now;
      in.tilt_deg = f.tilt_deg;
      in.agl_fused_m = f.agl_fused_m;
      in.vz_fused_mps = f.vz_fused_mps;
      in.vz_mps = f.vz_mps;
      in.az_imu1_mps2 = f.az_imu1_mps2;
      in.t_apogee_s = f.t_apogee_s;
      in.apogee_agl_m = f.apogee_agl_m;
      in.agl_ready = f.agl_ready ? 1 : 0;
      in.bmp1_altitude_m = rec.bmp390.altitude_m;
      in.imu1_altitude_m = rec.imu1.baro_alt_m;
      in.imu1_valid = rec.imu1.ok;
      in.bmp1_valid = rec.bmp390.ok;
      in.imu2_valid = rec.imu2.ok;

      FcOutputs out{};
      fc_step(s_ctx, in, out);

      if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);
      s_stat.stamp_ms = now;
      s_stat.state = out.state;
      s_stat.flags = out.flags;
      s_stat.t_to_apogee_s = out.t_to_apogee_s;
      s_stat.t_since_launch_s = out.t_since_launch_s;
      s_stat.airbrake_cmd_deg = out.airbrake_cmd_deg;
      s_stat.mach_cons = out.mach_cons;
      s_stat.tilt_deg = out.tilt_deg;
      if (s_mutex) xSemaphoreGive(s_mutex);
      vTaskDelayUntil(&last, pdMS_TO_TICKS(TELEM_PERIOD_MS));
    }
  }

  void fcStartTask()
  {
    if (!s_mutex)
      s_mutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(fc_task, "fc", 4096, nullptr, TASK_PRIO_LOGGER, nullptr, APP_CPU_NUM);
  }

  bool fcGetStatus(FcStatus &out)
  {
    if (s_mutex)
      xSemaphoreTake(s_mutex, portMAX_DELAY);
    out = s_stat;
    if (s_mutex)
      xSemaphoreGive(s_mutex);
    return true;
  }

  void fcSoftReset()
  {
    if (!s_mutex)
      s_mutex = xSemaphoreCreateMutex();
    if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);
    memset(&s_stat, 0, sizeof(s_stat));
    s_stat.state = FC_SAFE;
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_core_inited = false; // fc_task will re-init on next iteration
    if (s_mutex) xSemaphoreGive(s_mutex);
  }

} // namespace svc
