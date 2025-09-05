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
// !SECTION

//* -- Task --
// #if SERIAL_PLOTTER_MODE

static void task_logger(void *param) {
  const TickType_t period = pdMS_TO_TICKS(LOGGER_PERIOD_MS);
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    // VS Code Serial Plotter format: ">label:value,label:value"
    TelemetryRecord rec; telemetryGetLatest(rec);
    svc::FusedAlt f; svc::fusionGetAlt(f);
    bool agl_ready = f.agl_ready;

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
    Serial.print(",yaw_deg:"); Serial.print(f.yaw_deg, 2);
    Serial.print(",pitch_deg:"); Serial.print(f.pitch_deg, 2);
    Serial.print(",roll_deg:"); Serial.print(f.roll_deg, 2);
    Serial.print(",tilt_deg:"); Serial.print(f.tilt_deg, 2);
    Serial.print(",tilt_az_deg360:"); Serial.print(f.tilt_az_deg360, 2);
    Serial.print(",t_apogee_s:"); Serial.print(f.t_apogee_s, 3);
    Serial.print(",apogee_agl_m:"); Serial.print(f.apogee_agl_m, 2);
    Serial.println();

    vTaskDelayUntil(&last, period);
  }
}

// #endif

// #if PLOT_COMPARE_ACCEL
//     // Prefer telemetry snapshot when available
//     TelemetryRecord rec;
//     bool have_telem = false;
// #if MON_LOG_FROM_TELEM
//     have_telem = telemetry_get_latest(rec);
// #endif

//     imu1_reading_t u1; bool v1 = false;
//     imu2_reading_t u2; bool v2 = false;
//     if (have_telem) {
//       // IMU1 accel not in TelemetryImu1 (yet); fall back to direct getter
//       v1 = imu1_get(u1) && u1.valid;
//       // IMU2 accel is present in telemetry
//       v2 = (rec.hdr.present_flags & TP_IMU2) != 0;
//       if (v2) {
//         u2.accel_g[0] = rec.imu2.accel_g[0];
//         u2.accel_g[1] = rec.imu2.accel_g[1];
//         u2.accel_g[2] = rec.imu2.accel_g[2];
//       }
//     } else {
//       v1 = imu1_get(u1) && u1.valid;
//       v2 = imu2_get(u2) && u2.valid;
//     }

//     Serial.print(">");

//     auto emit_pair = [](const char* l1, float v1, const char* l2, float v2, bool &first){
//       if (!first) Serial.print(","); first = false;
//       Serial.print(l1); Serial.print(":"); Serial.print(v1, 6); Serial.print(",");
//       Serial.print(l2); Serial.print(":"); Serial.print(v2, 6);
//     };
//     auto emit_diff = [](const char* ld, float vd, bool &first){
//       if (!first) Serial.print(","); first = false;
//       Serial.print(ld); Serial.print(":"); Serial.print(vd, 6);
//     };

//     bool first = true;

//     if (PLOT_ACCEL_AXES_MASK & 0x1) {
//       float a1 = v1 ? u1.accel_g[0] : NAN;
//       float a2 = v2 ? u2.accel_g[0] : NAN;
//       emit_pair("imu1_ax_g", a1, "imu2_ax_g", a2, first);
// #if PLOT_INCLUDE_DIFF
//       emit_diff("diff_ax_g", (v1 && v2) ? (a1 - a2) : NAN, first);
// #endif
//     }
//     if (PLOT_ACCEL_AXES_MASK & 0x2) {
//       float a1 = v1 ? u1.accel_g[1] : NAN;
//       float a2 = v2 ? u2.accel_g[1] : NAN;
//       emit_pair("imu1_ay_g", a1, "imu2_ay_g", a2, first);
// #if PLOT_INCLUDE_DIFF
//       emit_diff("diff_ay_g", (v1 && v2) ? (a1 - a2) : NAN, first);
// #endif
//     }
//     if (PLOT_ACCEL_AXES_MASK & 0x4) {
//       float a1 = v1 ? u1.accel_g[2] : NAN;
//       float a2 = v2 ? u2.accel_g[2] : NAN;
//       emit_pair("imu1_az_g", a1, "imu2_az_g", a2, first);
// #if PLOT_INCLUDE_DIFF
//       emit_diff("diff_az_g", (v1 && v2) ? (a1 - a2) : NAN, first);
// #endif
//     }

//     Serial.println();
// #else
//     // Single-channel mode (backward-compatible)
//     float value = 0.0f; bool have = false;
//     switch (PLOT_SOURCE) {
//       case PLOT_SRC_IMU1_AX: case PLOT_SRC_IMU1_AY: case PLOT_SRC_IMU1_AZ: {
//         imu1_reading_t u; if (imu1_get(u) && u.valid) {
//           int idx = (PLOT_SOURCE == PLOT_SRC_IMU1_AX) ? 0 : (PLOT_SOURCE == PLOT_SRC_IMU1_AY ? 1 : 2);
//           value = u.accel_g[idx]; have = true; }
//         break; }
//       case PLOT_SRC_IMU2_AX: case PLOT_SRC_IMU2_AY: case PLOT_SRC_IMU2_AZ: {
//         // Prefer telemetry for IMU2
//         TelemetryRecord rec;
//         bool have_telem = false;
// #if MON_LOG_FROM_TELEM
//         have_telem = telemetry_get_latest(rec);
// #endif
//         if (have_telem && (rec.hdr.present_flags & TP_IMU2)) {
//           int idx = (PLOT_SOURCE == PLOT_SRC_IMU2_AX) ? 0 : (PLOT_SOURCE == PLOT_SRC_IMU2_AY ? 1 : 2);
//           value = rec.imu2.accel_g[idx]; have = true;
//         } else {
//           imu2_reading_t u2; if (imu2_get(u2) && u2.valid) {
//             int idx = (PLOT_SOURCE == PLOT_SRC_IMU2_AX) ? 0 : (PLOT_SOURCE == PLOT_SRC_IMU2_AY ? 1 : 2);
//             value = u2.accel_g[idx]; have = true; }
//         }
//         break; }
//       case PLOT_SRC_IMU2_GX: case PLOT_SRC_IMU2_GY: case PLOT_SRC_IMU2_GZ: {
//         TelemetryRecord rec;
//         bool have_telem = false;
// #if MON_LOG_FROM_TELEM
//         have_telem = telemetry_get_latest(rec);
// #endif
//         if (have_telem && (rec.hdr.present_flags & TP_IMU2)) {
//           int idx = (PLOT_SOURCE == PLOT_SRC_IMU2_GX) ? 0 : (PLOT_SOURCE == PLOT_SRC_IMU2_GY ? 1 : 2);
//           value = rec.imu2.gyro_dps[idx]; have = true;
//         } else {
//           imu2_reading_t u2; if (imu2_get(u2) && u2.valid) {
//             int idx = (PLOT_SOURCE == PLOT_SRC_IMU2_GX) ? 0 : (PLOT_SOURCE == PLOT_SRC_IMU2_GY ? 1 : 2);
//             value = u2.gyro_dps[idx]; have = true; }
//         }
//         break; }
//       default: have = false; break;
//     }
//     if (have) { Serial.print(">"); Serial.print(PLOT_VAR_LABEL); Serial.print(":"); Serial.print(value, 6); Serial.println(); }
// #endif
//     vTaskDelayUntil(&last, period);
//   }
// }

// #else
// static void print_header() {
//   bool first = true;
// #define HCOL(name) do { if (!first) Serial.print(","); Serial.print(name); first = false; } while (0)
// #if MON_ENABLE_TIME_MS
//   HCOL("time_ms");
// #endif
// #if MON_ENABLE_BMP1
//   HCOL("BMP1.temp_C"); HCOL("BMP1.press_hPa"); HCOL("BMP1.alt_m");
// #endif
// #if MON_ENABLE_IMU1_YPR
//   HCOL("IMU1.yaw_deg"); HCOL("IMU1.pitch_deg"); HCOL("IMU1.roll_deg");
// #endif
// #if MON_ENABLE_IMU1_ACCEL
//   HCOL("IMU1.ax_g"); HCOL("IMU1.ay_g"); HCOL("IMU1.az_g");
// #endif
// #if MON_ENABLE_IMU2_ACCEL
//   HCOL("IMU2.ax_g"); HCOL("IMU2.ay_g"); HCOL("IMU2.az_g");
// #endif
// #if MON_ENABLE_IMU2_GYRO
//   HCOL("IMU2.gx_dps"); HCOL("IMU2.gy_dps"); HCOL("IMU2.gz_dps");
// #endif
// #if MON_ENABLE_IMU2_TEMP
//   HCOL("IMU2.temp_C");
// #endif
//   Serial.println("");
// #undef HCOL
// }

// static void task_logger(void *param) {
//   const TickType_t period = pdMS_TO_TICKS(LOGGER_PERIOD_MS);
//   TickType_t last = xTaskGetTickCount();
//   uint32_t line = 0;
//   print_header();
//   for (;;) {
//     if ((line++ % MON_PRINT_HEADER_EVERY) == 0) print_header();

//     bool first = true;
// #define VAL(v) do { if (!first) Serial.print(","); Serial.printf("%g", (double)(v)); first = false; } while (0)

// #if MON_ENABLE_TIME_MS
//     VAL(millis());
// #endif

// #if MON_ENABLE_BMP1
//     {
//       bmp_reading_t r;
//       if (bmp390_get(r) && r.valid) {
//         VAL(r.temperature_c);
//         VAL(r.pressure_pa / 100.0f);
//         VAL(r.altitude_m);
//       } else {
//         VAL(NAN); VAL(NAN); VAL(NAN);
//       }
//     }
// #endif

// #if MON_ENABLE_IMU1_YPR || MON_ENABLE_IMU1_ACCEL
//     {
//       imu1_reading_t u;
//       if (imu1_get(u) && u.valid) {
// #if MON_ENABLE_IMU1_YPR
//         const float w = u.quat[0], x = u.quat[1], y = u.quat[2], z = u.quat[3];
//         const float nq = (w*w + x*x + y*y + z*z);
//         if (nq > 1e-6f) {
//           float yaw   = atan2f(2.0f*(x*y + w*z), 1.0f - 2.0f*(y*y + z*z)) * 57.2957795f;
//           float pitch = asinf(2.0f*(w*y - z*x)) * 57.2957795f;
//           float roll  = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y)) * 57.2957795f;
//           VAL(yaw); VAL(pitch); VAL(roll);
//         } else { VAL(NAN); VAL(NAN); VAL(NAN); }
// #endif
// #if MON_ENABLE_IMU1_ACCEL
//         VAL(u.accel_g[0]); VAL(u.accel_g[1]); VAL(u.accel_g[2]);
// #endif
//       } else {
// #if MON_ENABLE_IMU1_YPR
//         VAL(NAN); VAL(NAN); VAL(NAN);
// #endif
// #if MON_ENABLE_IMU1_ACCEL
//         VAL(NAN); VAL(NAN); VAL(NAN);
// #endif
//       }
//     }
// #endif

// #if MON_ENABLE_IMU2_ACCEL || MON_ENABLE_IMU2_GYRO || MON_ENABLE_IMU2_TEMP
//     {
//       imu2_reading_t u2;
//       if (imu2_get(u2) && u2.valid) {
// #if MON_ENABLE_IMU2_ACCEL
//         VAL(u2.accel_g[0]); VAL(u2.accel_g[1]); VAL(u2.accel_g[2]);
// #endif
// #if MON_ENABLE_IMU2_GYRO
//         VAL(u2.gyro_dps[0]); VAL(u2.gyro_dps[1]); VAL(u2.gyro_dps[2]);
// #endif
// #if MON_ENABLE_IMU2_TEMP
//         VAL(u2.temp_c);
// #endif
//       } else {
// #if MON_ENABLE_IMU2_ACCEL
//         VAL(NAN); VAL(NAN); VAL(NAN);
// #endif
// #if MON_ENABLE_IMU2_GYRO
//         VAL(NAN); VAL(NAN); VAL(NAN);
// #endif
// #if MON_ENABLE_IMU2_TEMP
//         VAL(NAN);
// #endif
//       }
//     }
// #endif

//     Serial.println("");
// #undef VAL
//     vTaskDelayUntil(&last, period);
//   }
// }
// #endif
// !SECTION

void loggerStartTask() {
  xTaskCreatePinnedToCore(task_logger, "logger", TASK_STACK_LOGGER, nullptr, TASK_PRIO_LOGGER, nullptr, APP_CPU_NUM);
}
