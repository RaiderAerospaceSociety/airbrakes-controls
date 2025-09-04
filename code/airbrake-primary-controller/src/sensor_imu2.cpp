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
#include "sensor_imu2.h"
#include "config/sensors_config.h"

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
    // Apply fixed orientation mapping to rocket body frame
    float ax_s = a.acceleration.x / G;
    float ay_s = a.acceleration.y / G;
    float az_s = a.acceleration.z / G;
    float gx_s = g.gyro.x * RAD2DEG;
    float gy_s = g.gyro.y * RAD2DEG;
    float gz_s = g.gyro.z * RAD2DEG;
    const float R[9] = { IMU2_R00, IMU2_R01, IMU2_R02,
                         IMU2_R10, IMU2_R11, IMU2_R12,
                         IMU2_R20, IMU2_R21, IMU2_R22 };
    auto rot = [&](float x, float y, float z, float &xo, float &yo, float &zo){
      xo = R[0]*x + R[1]*y + R[2]*z;
      yo = R[3]*x + R[4]*y + R[5]*z;
      zo = R[6]*x + R[7]*y + R[8]*z;
    };
    rot(ax_s, ay_s, az_s, r.accel_g[0], r.accel_g[1], r.accel_g[2]);
    rot(gx_s, gy_s, gz_s, r.gyro_dps[0], r.gyro_dps[1], r.gyro_dps[2]);
    r.temp_c = temp.temperature;
    r.valid = true;

    if (s_mutex) { xSemaphoreTake(s_mutex, portMAX_DELAY); s_latest = r; xSemaphoreGive(s_mutex); } else { s_latest = r; }
    vTaskDelayUntil(&last, period);
  }
}

void imu2StartTask() {
  xTaskCreatePinnedToCore(imu2_task, "imu2", 4096, nullptr, TASK_PRIO_BMP390, nullptr, APP_CPU_NUM);
}

bool imu2Get(imu2_reading_t &out) {
  bool v;
  if (s_mutex) xSemaphoreTake(s_mutex, portMAX_DELAY);
  out = s_latest;
  v = s_latest.valid;
  if (s_mutex) xSemaphoreGive(s_mutex);
  return v;
}
