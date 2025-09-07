// ===== Logger Task =====
// Brief: Emits labeled values for VS Code Serial Plotter.
// Refs: docs/monitoring.md, docs/signals.md
//* -- Includes --
// Logger task: periodically prints sensor readings
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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

static void task_logger(void *param) {
  const TickType_t period = pdMS_TO_TICKS(LOGGER_PERIOD_MS);
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    // VS Code Serial Plotter format: ">label:value,label:value"
    TelemetryRecord rec; telemetryGetLatest(rec);
    svc::FusedAlt f; svc::fusionGetAlt(f);
    bool agl_ready = f.agl_ready;

    // Teleplot vs Serial Plotter output
#if defined(TELEPLOT_MODE) && TELEPLOT_MODE
    // Teleplot: output one telemetry per line: name[:timestamp]:value
#if TELEPLOT_INCLUDE_TS
    #define TP_LINE(name, val, prec) do { \
      Serial.print(name); Serial.print(":"); \
      Serial.print((uint32_t)millis()); Serial.print(":"); \
      Serial.println((val), (prec)); \
    } while (0)
#else
    #define TP_LINE(name, val, prec) do { \
      Serial.print(">");Serial.print(name); Serial.print(":"); \
      Serial.println((val), (prec)); \
    } while (0)
#endif

#if defined(PLOT_SINGLE_ONLY) && PLOT_SINGLE_ONLY
    TP_LINE(PLOT_SINGLE_LABEL, (PLOT_SINGLE_EXPR), 3);
#else
    TP_LINE("bmp1_alt_m",          f.bmp1_alt_m, 3);
    TP_LINE("imu1_baro_alt_m",     f.imu1_alt_m, 3);
    TP_LINE("agl_bmp1_m",          agl_ready ? f.agl_bmp1_m     : NAN, 3);
    TP_LINE("agl_imu1_m",          agl_ready ? f.agl_imu1_m     : NAN, 3);
    TP_LINE("agl_fused_m",         agl_ready ? f.agl_fused_m    : NAN, 3);
    TP_LINE("vz_baro_mps",         agl_ready ? f.vz_mps         : NAN, 3);
#if FUSION_USE_ACC_INT
    TP_LINE("vz_acc_mps",          agl_ready ? f.vz_acc_mps     : NAN, 3);
#endif
    TP_LINE("vz_fused_mps",        agl_ready ? f.vz_fused_mps   : NAN, 3);
    TP_LINE("az_imu1_mps2",        f.az_imu1_mps2, 3);
    TP_LINE("temp_C",              f.temp_c, 3);
    TP_LINE("press_hPa",           f.press_hPa, 3);
    TP_LINE("sos_mps",             f.sos_mps, 3);
    TP_LINE("mach_vz",             f.mach_vz, 4);
    TP_LINE("sos_min_mps",         f.sos_min_mps, 3);
    TP_LINE("mach_cons",           f.mach_cons, 4);
    TP_LINE("tilt_deg",            f.tilt_deg, 2);
    TP_LINE("tilt_az_deg360",      f.tilt_az_deg360, 2);
    TP_LINE("t_apogee_s",          f.t_apogee_s, 3);
    TP_LINE("apogee_agl_m",        f.apogee_agl_m, 2);
#endif
    #undef TP_LINE
#else
    // Legacy VS Code Serial Plotter: single line with comma-separated pairs
    Serial.print(">bmp1_alt_m:"); Serial.print(f.bmp1_alt_m, 3);
    Serial.print(",imu1_baro_alt_m:"); Serial.print(f.imu1_alt_m, 3);
    Serial.print(",agl_bmp1_m:"); Serial.print(agl_ready ? f.agl_bmp1_m : NAN, 3);
    Serial.print(",agl_imu1_m:"); Serial.print(agl_ready ? f.agl_imu1_m : NAN, 3);
    Serial.print(",agl_fused_m:"); Serial.print(agl_ready ? f.agl_fused_m : NAN, 3);
    Serial.print(",vz_baro_mps:"); Serial.print(agl_ready ? f.vz_mps : NAN, 3);
#if FUSION_USE_ACC_INT
    Serial.print(",vz_acc_mps:"); Serial.print(agl_ready ? f.vz_acc_mps : NAN, 3);
#endif
    Serial.print(",vz_fused_mps:"); Serial.print(agl_ready ? f.vz_fused_mps : NAN, 3);
    Serial.print(",az_imu1_mps2:"); Serial.print(f.az_imu1_mps2, 3);
    Serial.print(",temp_C:"); Serial.print(f.temp_c, 3);
    Serial.print(",press_hPa:"); Serial.print(f.press_hPa, 3);
    Serial.print(",sos_mps:"); Serial.print(f.sos_mps, 3);
    Serial.print(",mach_vz:"); Serial.print(f.mach_vz, 4);
    Serial.print(",sos_min_mps:"); Serial.print(f.sos_min_mps, 3);
    Serial.print(",mach_cons:"); Serial.print(f.mach_cons, 4);
    Serial.print(",tilt_deg:"); Serial.print(f.tilt_deg, 2);
    Serial.print(",tilt_az_deg360:"); Serial.print(f.tilt_az_deg360, 2);
    Serial.print(",t_apogee_s:"); Serial.print(f.t_apogee_s, 3);
    Serial.print(",apogee_agl_m:"); Serial.print(f.apogee_agl_m, 2);
    Serial.println();
#endif

    // Debug feed (non-plotter): human-friendly CSV or multi-line block of FC state and flags
    svc::FcStatus st; svc::fcGetStatus(st);
    auto state_name = [](uint8_t s)->const char*{
      switch (s) {
        case svc::FC_SAFE: return "SAFE";
        case svc::FC_PREFLIGHT: return "PREFLIGHT";
        case svc::FC_ARMED_WAIT: return "ARMED_WAIT";
        case svc::FC_BOOST: return "BOOST";
        case svc::FC_POST_BURN_HOLD: return "POST_BURN_HOLD";
        case svc::FC_WINDOW: return "WINDOW";
        case svc::FC_DEPLOYED: return "DEPLOYED";
        case svc::FC_RETRACTING: return "RETRACTING";
        case svc::FC_LOCKED: return "LOCKED";
        case svc::FC_ABORT_LOCKOUT: return "ABORT_LOCKOUT";
        default: return "UNKNOWN";
      }
    };
    uint32_t ff = st.flags;
#if defined(TELEPLOT_MODE) && TELEPLOT_MODE && TELEPLOT_DEBUG_BLOCK
    // Multi-line, visually organized Teleplot logs
    Serial.print(">:DBG state: "); Serial.print(state_name(st.state)); Serial.print(" ("); Serial.print(st.state); Serial.println(")");
    Serial.print(">:DBG flags: 0x"); Serial.print(ff, HEX);
    Serial.print(" [IMU1:"); Serial.print((ff & svc::FCF_SENS_IMU1_OK) ? 1 : 0);
    Serial.print(" BMP1:"); Serial.print((ff & svc::FCF_SENS_BMP1_OK) ? 1 : 0);
    Serial.print(" IMU2:"); Serial.print((ff & svc::FCF_SENS_IMU2_OK) ? 1 : 0);
    Serial.print(" BARO_AGREE:"); Serial.print((ff & svc::FCF_BARO_AGREE) ? 1 : 0);
    Serial.print(" MACH_OK:"); Serial.print((ff & svc::FCF_MACH_OK) ? 1 : 0);
    Serial.print(" TILT_OK:"); Serial.print((ff & svc::FCF_TILT_OK) ? 1 : 0);
    Serial.print(" TILT_LATCH:"); Serial.print((ff & svc::FCF_TILT_LATCH) ? 1 : 0);
    Serial.print(" LIFTOFF_DET:"); Serial.print((ff & svc::FCF_LIFTOFF_DET) ? 1 : 0);
    Serial.print(" BURNOUT_DET:"); Serial.print((ff & svc::FCF_BURNOUT_DET) ? 1 : 0);
    Serial.println("]");
    Serial.print(">:DBG cmd_deg:"); Serial.print(st.airbrake_cmd_deg, 2);
    Serial.print(" tilt_deg:"); Serial.print(st.tilt_deg, 2);
    Serial.print(" mach_cons:"); Serial.print(st.mach_cons, 3);
    Serial.print(" t_to_apogee_s:"); Serial.print(st.t_to_apogee_s, 2);
    Serial.print(" t_since_launch_s:"); Serial.print(st.t_since_launch_s, 2);
    Serial.println();
#else
    // Single-line CSV debug
#if defined(TELEPLOT_MODE) && TELEPLOT_MODE
    Serial.print(">:");
#endif
    Serial.print("DBG,fc_state_str:"); Serial.print(state_name(st.state));
    Serial.print(",fc_state:"); Serial.print(st.state);
    Serial.print(",flags_hex:0x"); Serial.print(ff, HEX);
    Serial.print(",cmd_deg:"); Serial.print(st.airbrake_cmd_deg, 2);
    Serial.print(",mach_cons:"); Serial.print(st.mach_cons, 3);
    Serial.print(",tilt_deg:"); Serial.print(st.tilt_deg, 2);
    Serial.print(",t_to_apogee_s:"); Serial.print(st.t_to_apogee_s, 2);
    Serial.print(",t_since_launch_s:"); Serial.print(st.t_since_launch_s, 2);
    Serial.print(",SENS_IMU1_OK:"); Serial.print((ff & svc::FCF_SENS_IMU1_OK) ? 1 : 0);
    Serial.print(",SENS_BMP1_OK:"); Serial.print((ff & svc::FCF_SENS_BMP1_OK) ? 1 : 0);
    Serial.print(",SENS_IMU2_OK:"); Serial.print((ff & svc::FCF_SENS_IMU2_OK) ? 1 : 0);
    Serial.print(",BARO_AGREE:"); Serial.print((ff & svc::FCF_BARO_AGREE) ? 1 : 0);
    Serial.print(",MACH_OK:"); Serial.print((ff & svc::FCF_MACH_OK) ? 1 : 0);
    Serial.print(",TILT_OK:"); Serial.print((ff & svc::FCF_TILT_OK) ? 1 : 0);
    Serial.print(",TILT_LATCH:"); Serial.print((ff & svc::FCF_TILT_LATCH) ? 1 : 0);
    Serial.print(",LIFTOFF_DET:"); Serial.print((ff & svc::FCF_LIFTOFF_DET) ? 1 : 0);
    Serial.print(",BURNOUT_DET:"); Serial.print((ff & svc::FCF_BURNOUT_DET) ? 1 : 0);
    Serial.println();
#endif

    vTaskDelayUntil(&last, period);
  }
}

// !SECTION

void loggerStartTask() {
  xTaskCreatePinnedToCore(task_logger, "logger", TASK_STACK_LOGGER, nullptr, TASK_PRIO_LOGGER, nullptr, APP_CPU_NUM);
}
