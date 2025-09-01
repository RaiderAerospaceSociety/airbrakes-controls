// IMU1: USFSMAX (MAX32660 + MMC5983) minimal reader over I2C
#pragma once

#include <Arduino.h>

typedef struct {
  float quat[4];     // w,x,y,z
  float accel_g[3];  // ax, ay, az in g
  float pressure_pa; // internal baro pressure (Pa)
  float altitude_m;  // internal baro altitude (m)
  bool  valid;
} imu1_reading_t;

void imu1_start_task();
bool imu1_get(imu1_reading_t &out);
