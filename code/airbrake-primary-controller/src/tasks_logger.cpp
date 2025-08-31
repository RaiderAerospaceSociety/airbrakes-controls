// ANCHOR: Overview
// SECTION - Includes ---------------------------------------------------------
// Logger task: periodically prints sensor readings
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "app_config.h"
#include "logging.h"
#include "sensors_bmp390.h"
#include "sensors_imu1.h"
#include "sensors_imu2.h"
// !SECTION
extern float heading[2];
extern float angle[2][2];

// SECTION - Task -------------------------------------------------------------
#if SERIAL_PLOTTER_MODE

static void task_logger(void *param) {
  const TickType_t period = pdMS_TO_TICKS(LOGGER_PERIOD_MS);
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    float value = 0.0f;
    bool have = false;

    switch (PLOT_SOURCE) {
      case PLOT_SRC_IMU1_AX:
      case PLOT_SRC_IMU1_AY:
      case PLOT_SRC_IMU1_AZ: {
        imu1_reading_t u;
        if (imu1_get(u) && u.valid) {
          int idx = (PLOT_SOURCE == PLOT_SRC_IMU1_AX) ? 0 : (PLOT_SOURCE == PLOT_SRC_IMU1_AY ? 1 : 2);
          value = u.accel_g[idx];
          have = true;
        }
        break;
      }
      case PLOT_SRC_IMU2_AX:
      case PLOT_SRC_IMU2_AY:
      case PLOT_SRC_IMU2_AZ: {
        imu2_reading_t u2;
        if (imu2_get(u2) && u2.valid) {
          int idx = (PLOT_SOURCE == PLOT_SRC_IMU2_AX) ? 0 : (PLOT_SOURCE == PLOT_SRC_IMU2_AY ? 1 : 2);
          value = u2.accel_g[idx];
          have = true;
        }
        break;
      }
      case PLOT_SRC_IMU2_GX:
      case PLOT_SRC_IMU2_GY:
      case PLOT_SRC_IMU2_GZ: {
        imu2_reading_t u2;
        if (imu2_get(u2) && u2.valid) {
          int idx = (PLOT_SOURCE == PLOT_SRC_IMU2_GX) ? 0 : (PLOT_SOURCE == PLOT_SRC_IMU2_GY ? 1 : 2);
          value = u2.gyro_dps[idx];
          have = true;
        }
        break;
      }
      default:
        have = false;
        break;
    }

    if (have) {
      Serial.print(">");
      Serial.print(PLOT_VAR_LABEL);
      Serial.print(":");
      Serial.print(value, 6);
      Serial.println();
    }

    vTaskDelayUntil(&last, period);
  }
}

#else
static void print_header() {
  bool first = true;
#define HCOL(name) do { if (!first) Serial.print(","); Serial.print(name); first = false; } while (0)
#if MON_ENABLE_TIME_MS
  HCOL("time_ms");
#endif
#if MON_ENABLE_BMP1
  HCOL("BMP1.temp_C"); HCOL("BMP1.press_hPa"); HCOL("BMP1.alt_m");
#endif
#if MON_ENABLE_IMU1_YPR
  HCOL("IMU1.yaw_deg"); HCOL("IMU1.pitch_deg"); HCOL("IMU1.roll_deg");
#endif
#if MON_ENABLE_IMU1_ACCEL
  HCOL("IMU1.ax_g"); HCOL("IMU1.ay_g"); HCOL("IMU1.az_g");
#endif
#if MON_ENABLE_IMU2_ACCEL
  HCOL("IMU2.ax_g"); HCOL("IMU2.ay_g"); HCOL("IMU2.az_g");
#endif
#if MON_ENABLE_IMU2_GYRO
  HCOL("IMU2.gx_dps"); HCOL("IMU2.gy_dps"); HCOL("IMU2.gz_dps");
#endif
#if MON_ENABLE_IMU2_TEMP
  HCOL("IMU2.temp_C");
#endif
  Serial.println("");
#undef HCOL
}

static void task_logger(void *param) {
  const TickType_t period = pdMS_TO_TICKS(LOGGER_PERIOD_MS);
  TickType_t last = xTaskGetTickCount();
  uint32_t line = 0;
  print_header();
  for (;;) {
    if ((line++ % MON_PRINT_HEADER_EVERY) == 0) print_header();

    bool first = true;
#define VAL(v) do { if (!first) Serial.print(","); Serial.printf("%g", (double)(v)); first = false; } while (0)

#if MON_ENABLE_TIME_MS
    VAL(millis());
#endif

#if MON_ENABLE_BMP1
    {
      bmp_reading_t r;
      if (bmp390_get(r) && r.valid) {
        VAL(r.temperature_c);
        VAL(r.pressure_pa / 100.0f);
        VAL(r.altitude_m);
      } else {
        VAL(NAN); VAL(NAN); VAL(NAN);
      }
    }
#endif

#if MON_ENABLE_IMU1_YPR || MON_ENABLE_IMU1_ACCEL
    {
      imu1_reading_t u;
      if (imu1_get(u) && u.valid) {
#if MON_ENABLE_IMU1_YPR
        const float w = u.quat[0], x = u.quat[1], y = u.quat[2], z = u.quat[3];
        const float nq = (w*w + x*x + y*y + z*z);
        if (nq > 1e-6f) {
          float yaw   = atan2f(2.0f*(x*y + w*z), 1.0f - 2.0f*(y*y + z*z)) * 57.2957795f;
          float pitch = asinf(2.0f*(w*y - z*x)) * 57.2957795f;
          float roll  = atan2f(2.0f*(w*x + y*z), 1.0f - 2.0f*(x*x + y*y)) * 57.2957795f;
          VAL(yaw); VAL(pitch); VAL(roll);
        } else { VAL(NAN); VAL(NAN); VAL(NAN); }
#endif
#if MON_ENABLE_IMU1_ACCEL
        VAL(u.accel_g[0]); VAL(u.accel_g[1]); VAL(u.accel_g[2]);
#endif
      } else {
#if MON_ENABLE_IMU1_YPR
        VAL(NAN); VAL(NAN); VAL(NAN);
#endif
#if MON_ENABLE_IMU1_ACCEL
        VAL(NAN); VAL(NAN); VAL(NAN);
#endif
      }
    }
#endif

#if MON_ENABLE_IMU2_ACCEL || MON_ENABLE_IMU2_GYRO || MON_ENABLE_IMU2_TEMP
    {
      imu2_reading_t u2;
      if (imu2_get(u2) && u2.valid) {
#if MON_ENABLE_IMU2_ACCEL
        VAL(u2.accel_g[0]); VAL(u2.accel_g[1]); VAL(u2.accel_g[2]);
#endif
#if MON_ENABLE_IMU2_GYRO
        VAL(u2.gyro_dps[0]); VAL(u2.gyro_dps[1]); VAL(u2.gyro_dps[2]);
#endif
#if MON_ENABLE_IMU2_TEMP
        VAL(u2.temp_c);
#endif
      } else {
#if MON_ENABLE_IMU2_ACCEL
        VAL(NAN); VAL(NAN); VAL(NAN);
#endif
#if MON_ENABLE_IMU2_GYRO
        VAL(NAN); VAL(NAN); VAL(NAN);
#endif
#if MON_ENABLE_IMU2_TEMP
        VAL(NAN);
#endif
      }
    }
#endif

    Serial.println("");
#undef VAL
    vTaskDelayUntil(&last, period);
  }
}
#endif
// !SECTION

void logger_start_task() {
  xTaskCreatePinnedToCore(task_logger, "logger", TASK_STACK_LOGGER, nullptr, TASK_PRIO_LOGGER, nullptr, APP_CPU_NUM);
}
