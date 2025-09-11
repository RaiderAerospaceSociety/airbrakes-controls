// ===== Logger Task =====
// Brief: Emits labeled values for VS Code Serial Plotter.
// Refs: docs/monitoring.md, docs/signals.md
//* -- Includes --
// Logger task: periodically prints sensor readings
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "app_config.h"
#include "logging.h"
#include "sensor_bmp390.h"
#include "sensor_imu1.h"
#include "sensor_imu2.h"
#include "telemetry.h"
#include "services/fusion.h"
#include "services/fc.h"
// !SECTION

//* -- Task --

static void task_logger(void *param)
{
  TickType_t last = xTaskGetTickCount();
  for (;;)
  {
    // Minimal custom visualization mode: output only "tilt_deg:<angle>"
#if SERIAL_DATA_ENABLE && (defined(VIS_TILT_ONLY_MODE) && VIS_TILT_ONLY_MODE)
    svc::FusedAlt f_min;
    svc::fusionGetAlt(f_min);
    Serial.print("tilt_deg:");
    Serial.println(f_min.tilt_deg, 2);
    vTaskDelayUntil(&last, pdMS_TO_TICKS(LOGGER_PERIOD_MS));
    continue;
#endif
    // Unified one-line output with key:value pairs
    TelemetryRecord rec;
    telemetryGetLatest(rec);
    svc::FusedAlt f;
    svc::fusionGetAlt(f);
    bool agl_ready = f.agl_ready;

#if SERIAL_DATA_ENABLE && MON_UNIFIED_OUTPUT
    auto begin_line = []()
    {
      Serial.print("ts:");
      Serial.print((uint32_t)millis());
    };
    auto kv_f = [](const char *key, float val, int prec)
    {
      Serial.print(", ");
      Serial.print(key);
      Serial.print(":");
      if (isnan(val))
        Serial.print("nan");
      else
        Serial.print(val, prec);
    };
    auto kv_i = [](const char *key, int32_t val)
    {
      Serial.print(", ");
      Serial.print(key);
      Serial.print(":");
      Serial.print(val);
    };

    begin_line();
#if VIS_TILT_ONLY_MODE
    kv_f("tilt_deg", f.tilt_deg, 2);
#else
#if MON_PROFILE_FULL
    // Extended set
    kv_f("bmp1_alt_m", f.bmp1_alt_m, 3);
    kv_f("imu1_baro_alt_m", f.imu1_alt_m, 3);
    kv_f("agl_bmp1_m", agl_ready ? f.agl_bmp1_m : NAN, 3);
    kv_f("agl_imu1_m", agl_ready ? f.agl_imu1_m : NAN, 3);
    kv_f("agl_fused_m", agl_ready ? f.agl_fused_m : NAN, 3);
    kv_f("vz_baro_mps", agl_ready ? f.vz_mps : NAN, 3);
#if FUSION_USE_ACC_INT
    kv_f("vz_acc_mps", agl_ready ? f.vz_acc_mps : NAN, 3);
#endif
    kv_f("vz_fused_mps", agl_ready ? f.vz_fused_mps : NAN, 3);
    kv_f("az_imu1_mps2", f.az_imu1_mps2, 3);
    kv_f("temp_C", f.temp_c, 3);
    kv_f("press_hPa", f.press_hPa, 3);
    kv_f("sos_mps", f.sos_mps, 3);
    kv_f("sos_min_mps", f.sos_min_mps, 3);
    kv_f("mach_vz", f.mach_vz, 4);
    kv_f("tilt_az_deg360", f.tilt_az_deg360, 2);
    kv_f("t_apogee_s", f.t_apogee_s, 3);
    kv_f("apogee_agl_m", f.apogee_agl_m, 2);
#endif // MON_PROFILE_FULL
    // Core minimal set
    kv_f("tilt_deg", f.tilt_deg, 2);
    kv_f("mach_cons", f.mach_cons, 4);
    kv_f("cmd_deg", (float)rec.ctl.airbrake_cmd_deg, 2);
    kv_i("fc_state", (int32_t)rec.sys.fc_state);
    // Flags
    {
      uint32_t ff = rec.sys.fc_flags;
      kv_i("tilt_ok", (ff & svc::FCF_TILT_OK) ? 1 : 0);
      kv_i("tilt_lock", (ff & svc::FCF_TILT_LATCH) ? 1 : 0);
    }
#endif // VIS_TILT_ONLY_MODE
    Serial.println();
#endif // SERIAL_DATA_ENABLE && MON_UNIFIED_OUTPUT

    // Status/flags stream (considered DATA, not DEBUG): emits periodically
#if SERIAL_DATA_ENABLE && MON_DEBUG_BLOCK && !(defined(VIS_TILT_ONLY_MODE) && VIS_TILT_ONLY_MODE)
    svc::FcStatus st;
    svc::fcGetStatus(st);
    auto state_name = [](uint8_t s) -> const char *
    {
      switch (s)
      {
      case svc::FC_SAFE:
        return "SAFE";
      case svc::FC_PREFLIGHT:
        return "PREFLIGHT";
      case svc::FC_ARMED_WAIT:
        return "ARMED_WAIT";
      case svc::FC_BOOST:
        return "BOOST";
      case svc::FC_POST_BURN_HOLD:
        return "POST_BURN_HOLD";
      case svc::FC_WINDOW:
        return "WINDOW";
      case svc::FC_DEPLOYED:
        return "DEPLOYED";
      case svc::FC_RETRACTING:
        return "RETRACTING";
      case svc::FC_LOCKED:
        return "LOCKED";
      case svc::FC_ABORT_LOCKOUT:
        return "ABORT_LOCKOUT";
      default:
        return "UNKNOWN";
      }
    };
    uint32_t ff = st.flags;
#if defined(TELEPLOT_MODE) && TELEPLOT_MODE && TELEPLOT_DEBUG_BLOCK
    // Multi-line, visually organized Teleplot logs
    Serial.print(">:DBG state: ");
    Serial.print(state_name(st.state));
    Serial.print(" (");
    Serial.print(st.state);
    Serial.println(")");
    Serial.print(">:DBG flags: 0x");
    Serial.print(ff, HEX);
    Serial.print(" [IMU1:");
    Serial.print((ff & svc::FCF_SENS_IMU1_OK) ? 1 : 0);
    Serial.print(" BMP1:");
    Serial.print((ff & svc::FCF_SENS_BMP1_OK) ? 1 : 0);
    Serial.print(" IMU2:");
    Serial.print((ff & svc::FCF_SENS_IMU2_OK) ? 1 : 0);
    Serial.print(" BARO_AGREE:");
    Serial.print((ff & svc::FCF_BARO_AGREE) ? 1 : 0);
    Serial.print(" MACH_OK:");
    Serial.print((ff & svc::FCF_MACH_OK) ? 1 : 0);
    Serial.print(" TILT_OK:");
    Serial.print((ff & svc::FCF_TILT_OK) ? 1 : 0);
    Serial.print(" TILT_LATCH:");
    Serial.print((ff & svc::FCF_TILT_LATCH) ? 1 : 0);
    Serial.print(" LIFTOFF_DET:");
    Serial.print((ff & svc::FCF_LIFTOFF_DET) ? 1 : 0);
    Serial.print(" BURNOUT_DET:");
    Serial.print((ff & svc::FCF_BURNOUT_DET) ? 1 : 0);
    Serial.println("]");
    Serial.print(">:DBG cmd_deg:");
    Serial.print(st.airbrake_cmd_deg, 2);
    Serial.print(" tilt_deg:");
    Serial.print(st.tilt_deg, 2);
    Serial.print(" mach_cons:");
    Serial.print(st.mach_cons, 3);
    Serial.print(" t_to_apogee_s:");
    Serial.print(st.t_to_apogee_s, 2);
    Serial.print(" t_since_launch_s:");
    Serial.print(st.t_since_launch_s, 2);
    Serial.println();
#else // MON_DEBUG_BLOCK
    // Single-line CSV debug
#if defined(TELEPLOT_MODE) && TELEPLOT_MODE
    Serial.print(">:");
#endif // MON_DEBUG_BLOCK
    Serial.print("DBG,fc_state_str:");
    Serial.print(state_name(st.state));
    Serial.print(",fc_state:");
    Serial.print(st.state);
    Serial.print(",flags_hex:0x");
    Serial.print(ff, HEX);
    Serial.print(",cmd_deg:");
    Serial.print(st.airbrake_cmd_deg, 2);
    Serial.print(",mach_cons:");
    Serial.print(st.mach_cons, 3);
    Serial.print(",tilt_deg:");
    Serial.print(st.tilt_deg, 2);
    Serial.print(",t_to_apogee_s:");
    Serial.print(st.t_to_apogee_s, 2);
    Serial.print(",t_since_launch_s:");
    Serial.print(st.t_since_launch_s, 2);
    Serial.print(",SENS_IMU1_OK:");
    Serial.print((ff & svc::FCF_SENS_IMU1_OK) ? 1 : 0);
    Serial.print(",SENS_BMP1_OK:");
    Serial.print((ff & svc::FCF_SENS_BMP1_OK) ? 1 : 0);
    Serial.print(",SENS_IMU2_OK:");
    Serial.print((ff & svc::FCF_SENS_IMU2_OK) ? 1 : 0);
    Serial.print(",BARO_AGREE:");
    Serial.print((ff & svc::FCF_BARO_AGREE) ? 1 : 0);
    Serial.print(",MACH_OK:");
    Serial.print((ff & svc::FCF_MACH_OK) ? 1 : 0);
    Serial.print(",TILT_OK:");
    Serial.print((ff & svc::FCF_TILT_OK) ? 1 : 0);
    Serial.print(",TILT_LATCH:");
    Serial.print((ff & svc::FCF_TILT_LATCH) ? 1 : 0);
    Serial.print(",LIFTOFF_DET:");
    Serial.print((ff & svc::FCF_LIFTOFF_DET) ? 1 : 0);
    Serial.print(",BURNOUT_DET:");
    Serial.print((ff & svc::FCF_BURNOUT_DET) ? 1 : 0);
    Serial.println();
#endif

#endif // SERIAL_DATA_ENABLE

    vTaskDelayUntil(&last, pdMS_TO_TICKS(LOGGER_PERIOD_MS));
  }
}

// !SECTION

void loggerStartTask()
{
  xTaskCreatePinnedToCore(task_logger, "logger", TASK_STACK_LOGGER, nullptr, TASK_PRIO_LOGGER, nullptr, APP_CPU_NUM);
}
