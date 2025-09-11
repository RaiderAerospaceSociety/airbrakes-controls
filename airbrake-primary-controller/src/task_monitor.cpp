// ===== Monitoring Task =====
// Brief: Emits either Visualizer (key:value) or Human (fixed-width) lines.
// Refs: docs/monitoring.md, docs/signals.md
//* -- Includes --
// Logger task: periodically prints sensor readings
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "app_config.h"
#include "logging.h"
#include "sensor_bmp1.h"
#include "sensor_imu1.h"
#include "sensor_imu2.h"
#include "telemetry.h"
#include "services/fusion.h"
#include "services/fc.h"
// !SECTION

//* -- Task --

static void task_monitor(void *param)
{
  TickType_t last = xTaskGetTickCount();
  for (;;)
  {
    // Fetch data used for monitoring lines
#if SERIAL_DATA_ENABLE
    TelemetryRecord rec;
    telemetryGetLatest(rec);
    svc::FusedAlt f;
    svc::fusionGetAlt(f);
    bool agl_ready = f.agl_ready;

    // Mode 0: Visualizer (key:value)
#if (MON_MODE == 0)
    auto kv_f = [](const char *key, float val, int prec)
    {
      Serial.print(", "); Serial.print(key); Serial.print(":");
      if (isnan(val)) Serial.print("nan"); else Serial.print(val, prec);
    };
    auto kv_i = [](const char *key, int32_t val)
    {
      Serial.print(", "); Serial.print(key); Serial.print(":"); Serial.print(val);
    };
    if (MON_INCLUDE_TS) { Serial.print("ts:"); Serial.print((uint32_t)millis()); }
    // Core fused values used by FC decisions
    kv_f("tilt_deg", f.tilt_deg, 2);
    kv_f("mach_cons", f.mach_cons, 4);
    kv_f("cmd_deg", (float)rec.ctl.airbrake_cmd_deg, 2);
    kv_i("fc_state", (int32_t)rec.sys.fc_state);
    kv_i("fc_flags", (int32_t)rec.sys.fc_flags);
    {
      uint32_t ff = rec.sys.fc_flags;
      kv_i("tilt_ok", (ff & svc::FCF_TILT_OK) ? 1 : 0);
      kv_i("tilt_lock", (ff & svc::FCF_TILT_LATCH) ? 1 : 0);
    }
    // Optional fusion sub-values for verification
#if MON_SHOW_FUSION_PARTS
    kv_f("agl_fused_m", agl_ready ? f.agl_fused_m : NAN, 3);
    kv_f("agl_bmp1_m", agl_ready ? f.agl_bmp1_m : NAN, 3);
    kv_f("agl_imu1_m", agl_ready ? f.agl_imu1_m : NAN, 3);
    kv_f("vz_fused_mps", agl_ready ? f.vz_fused_mps : NAN, 3);
    kv_f("vz_baro_mps", agl_ready ? f.vz_mps : NAN, 3);
#if FUSION_USE_ACC_INT
    kv_f("vz_acc_mps", agl_ready ? f.vz_acc_mps : NAN, 3);
#endif
    kv_f("temp_C", f.temp_c, 2);
    kv_f("press_hPa", f.press_hPa, 1);
    kv_f("sos_min_mps", f.sos_min_mps, 2);
#endif // MON_SHOW_FUSION_PARTS
    Serial.println();
#elif (MON_MODE == 1)
    // Mode 1: Human (fixed-width, signed-aware)
    svc::FcStatus st; svc::fcGetStatus(st);
    auto state_name = [](uint8_t s) -> const char *
    {
      switch (s) {
        case svc::FC_SAFE: return "SAFE";
        case svc::FC_PREFLIGHT: return "PREFLIGHT";
        case svc::FC_ARMED_WAIT: return "ARMED_WAIT";
        case svc::FC_BOOST: return "BOOST";
        case svc::FC_POST_BURN_HOLD: return "POST_HOLD";
        case svc::FC_WINDOW: return "WINDOW";
        case svc::FC_DEPLOYED: return "DEPLOYED";
        case svc::FC_RETRACTING: return "RETRACT";
        case svc::FC_LOCKED: return "LOCKED";
        case svc::FC_ABORT_LOCKOUT: return "ABORT";
        default: return "UNKNOWN";
      }
    };
    uint32_t ff = rec.sys.fc_flags;
    int mach_ok = (ff & svc::FCF_MACH_OK) ? 1 : 0;
    int tilt_ok = (ff & svc::FCF_TILT_OK) ? 1 : 0;
    int tilt_lock = (ff & svc::FCF_TILT_LATCH) ? 1 : 0;
    if (MON_INCLUDE_TS) Serial.printf("%08lu ", (uint32_t)millis());
    Serial.printf("%-10s ", state_name(rec.sys.fc_state));
    Serial.printf("M:%d T:%d L:%d ", mach_ok, tilt_ok, tilt_lock);
    Serial.printf("cmd:%+05.1f ", (float)rec.ctl.airbrake_cmd_deg);
    Serial.printf("tilt:%+06.2f ", f.tilt_deg);
    Serial.printf("mach:%0.3f ", f.mach_cons);
    Serial.printf("vz:%+07.2f ", f.vz_fused_mps);
    Serial.printf("agl:%+07.2f", f.agl_fused_m);
    Serial.println();
#endif // MON_MODE
#endif // SERIAL_DATA_ENABLE

    vTaskDelayUntil(&last, pdMS_TO_TICKS(LOGGER_PERIOD_MS));
  }
}

// !SECTION

void monitorStartTask()
{
  xTaskCreatePinnedToCore(task_monitor, "monitor", TASK_STACK_LOGGER, nullptr, TASK_PRIO_LOGGER, nullptr, APP_CPU_NUM);
}
