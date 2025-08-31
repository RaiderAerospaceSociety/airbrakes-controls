// IMU2: MPU6050 task using Adafruit library
#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "pins.h"
#include "app_config.h"
#include "logging.h"
#include "bus.h"
#include "sensors_imu2.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

static SemaphoreHandle_t s_mutex = nullptr; // protect local snapshot
static imu2_reading_t    s_latest = {0};
static Adafruit_MPU6050  s_mpu;

static void imu2_task(void *param) {
  if (!s_mutex) s_mutex = xSemaphoreCreateMutex();

  bool ok = false;
  if (g_i2c_mutex) xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
  ok = s_mpu.begin(0x68, &Wire);
  if (g_i2c_mutex) xSemaphoreGive(g_i2c_mutex);
  if (!ok) {
    LOGLN("IMU2 (MPU6050) not found at 0x68");
    // Try alternate address 0x69
    if (g_i2c_mutex) xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
    ok = s_mpu.begin(0x69, &Wire);
    if (g_i2c_mutex) xSemaphoreGive(g_i2c_mutex);
  }
  if (!ok) {
    LOGLN("IMU2 (MPU6050) init failed; task exiting");
    vTaskDelete(NULL);
    return;
  }

  // Configure ranges and filter
  s_mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  s_mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  s_mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  LOGLN("IMU2 (MPU6050) initialized");

  const TickType_t period = pdMS_TO_TICKS(IMU2_PERIOD_MS);
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    sensors_event_t a, g, temp;
    if (g_i2c_mutex) xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
    s_mpu.getEvent(&a, &g, &temp);
    if (g_i2c_mutex) xSemaphoreGive(g_i2c_mutex);

    imu2_reading_t r;
    // Convert m/s^2 to g, rad/s to deg/s
    const float G = 9.80665f;
    const float RAD2DEG = 57.2957795f;
    r.accel_g[0] = a.acceleration.x / G;
    r.accel_g[1] = a.acceleration.y / G;
    r.accel_g[2] = a.acceleration.z / G;
    r.gyro_dps[0] = g.gyro.x * RAD2DEG;
    r.gyro_dps[1] = g.gyro.y * RAD2DEG;
    r.gyro_dps[2] = g.gyro.z * RAD2DEG;
    r.temp_c = temp.temperature;
    r.valid = true;

    if (s_mutex) { xSemaphoreTake(s_mutex, portMAX_DELAY); s_latest = r; xSemaphoreGive(s_mutex); } else { s_latest = r; }
    vTaskDelayUntil(&last, period);
  }
}

void imu2_start_task() {
  xTaskCreatePinnedToCore(imu2_task, "imu2", 4096, nullptr, TASK_PRIO_BMP390, nullptr, APP_CPU_NUM);
}

bool imu2_get(imu2_reading_t &out) {
  bool v;
  if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);
  out = s_latest;
  v = s_latest.valid;
  if (s_mutex) xSemaphoreGive(s_mutex);
  return v;
}

