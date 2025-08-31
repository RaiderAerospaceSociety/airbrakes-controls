// IMU2: MPU6050 reader over I2C (Adafruit library)
#pragma once

#include <Arduino.h>

typedef struct {
  float accel_g[3];   // ax, ay, az in g
  float gyro_dps[3];  // gx, gy, gz in deg/s
  float temp_c;       // temperature in C
  bool  valid;
} imu2_reading_t;

void imu2_start_task();
bool imu2_get(imu2_reading_t &out);

