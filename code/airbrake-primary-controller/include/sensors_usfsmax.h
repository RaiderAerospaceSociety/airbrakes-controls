// USFSMAX (MAX32660 + MMC5983) minimal reader over I2C
#pragma once

#include <Arduino.h>

typedef struct {
  float quat[4];     // w,x,y,z
  float accel_g[3];  // ax, ay, az in g
  bool  valid;
} usfs_reading_t;

void usfsmax_start_task();
bool usfsmax_get(usfs_reading_t &out);
