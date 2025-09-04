// ANCHOR: Overview
// SECTION - Includes ---------------------------------------------------------
// USFSMAX reader using upstream library (USFSMAX + I2Cdev)
#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "pins.h"
#include "app_config.h"
#include "logging.h"
#include "bus.h"
#include "sensor_imu1.h"

// SECTION - Library Bridge ---------------------------------------------------
#include <USFSMAX.h>
// !SECTION

extern float qt[2][4];
extern int16_t accADC[2][3];
extern float g_per_count;
extern float heading[2];
extern float angle[2][2];
extern int32_t baroADC[2];

// SECTION - Module Globals ---------------------------------------------------
static SemaphoreHandle_t s_usfs_mutex = nullptr; // protect local snapshot
static imu1_reading_t s_latest = {0};

static I2Cdev s_i2c(&Wire);
static USFSMAX s_usfs(&s_i2c, 0);
// !SECTION

static void usfs_task(void *param) {
  if (!s_usfs_mutex) s_usfs_mutex = xSemaphoreCreateMutex();

  // DRDY is not used; we poll at a fixed rate

  if (g_i2c_mutex) xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
  Wire.setClock(100000); // 100kHz for configuration
  if (g_i2c_mutex) xSemaphoreGive(g_i2c_mutex);

  s_usfs.init_USFSMAX();

  if (g_i2c_mutex) xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
  Wire.setClock(I2C_CLOCK);
  if (g_i2c_mutex) xSemaphoreGive(g_i2c_mutex);

  LOGLN("IMU1 (USFSMAX) initialized (library)");

  const TickType_t period = pdMS_TO_TICKS(USFS_PERIOD_MS);
  TickType_t last = xTaskGetTickCount();
  // SECTION - Main Task Loop -------------------------------------------------
  static float last_pressure_pa = NAN;
  static float last_altitude_m  = NAN;
  for (;;) {
    // Poll at a fixed rate; no DRDY gating
    // Follow example: read event status to optimize what to fetch
    uint8_t evt = 0;
    s_i2c.readBytes(MAX32660_SLV_ADDR, COMBO_DRDY_STAT, 1, &evt);

    // Bits: 0x01 Gyro, 0x02 Acc, 0x04 Mag, 0x08 Baro, 0x10 Quat
    uint8_t call_sensors = evt & 0x0F;
    switch (call_sensors) {
      case 0x01:
      case 0x02:
      case 0x03:
        s_usfs.GyroAccel_getADC();
        break;
      case 0x07:
      case 0x0B:
      case 0x0F:
        s_usfs.GyroAccelMagBaro_getADC();
        break;
      case 0x0C:
        s_usfs.MagBaro_getADC();
        break;
      case 0x04:
        s_usfs.MAG_getADC();
        break;
      case 0x08:
        s_usfs.BARO_getADC();
        break;
      default:
        // No combined sensor flags; still attempt to read accel to keep it fresh
        s_usfs.ACC_getADC();
        break;
    }

    if (evt & 0x10) {
      // New quaternion available
      s_usfs.getQUAT();
      // Also fetch Euler for visibility/debug
      s_usfs.getEULER();
    }

    // SECTION - Snapshot Build ----------------------------------------------
    imu1_reading_t r;
    r.quat[0] = qt[0][0];
    r.quat[1] = qt[0][1];
    r.quat[2] = qt[0][2];
    r.quat[3] = qt[0][3];
    r.accel_g[0] = accADC[0][0] * g_per_count;
    r.accel_g[1] = accADC[0][1] * g_per_count;
    r.accel_g[2] = accADC[0][2] * g_per_count;
    // Internal baro sample: update only when a new BARO event was indicated
    if (evt & 0x08) {
      // LPS22HB output: 4096 LSB/hPa => 100/4096 Pa per count
      last_pressure_pa = ((float)baroADC[0]) * (100.0f / 4096.0f);
      last_altitude_m  = 44330.0f * (1.0f - pow((last_pressure_pa / 100.0f) / SEALEVELPRESSURE_HPA, 0.1903f));
    }
    r.pressure_pa = last_pressure_pa;
    r.altitude_m  = last_altitude_m;
    r.valid   = true;
    if (s_usfs_mutex) {
      xSemaphoreTake(s_usfs_mutex, portMAX_DELAY);
      s_latest = r;
      xSemaphoreGive(s_usfs_mutex);
    } else {
      s_latest = r;
    }
    // !SECTION
    vTaskDelayUntil(&last, period);
  }
}
// !SECTION

void imu1_start_task() {
  xTaskCreatePinnedToCore(usfs_task, "usfsmax", 4096, nullptr, TASK_PRIO_BMP390, nullptr, APP_CPU_NUM);
}

bool imu1_get(imu1_reading_t &out) {
  bool v;
  if (s_usfs_mutex) xSemaphoreTake(s_usfs_mutex, portMAX_DELAY);
  out = s_latest;
  v = s_latest.valid;
  if (s_usfs_mutex) xSemaphoreGive(s_usfs_mutex);
  return v;
}
