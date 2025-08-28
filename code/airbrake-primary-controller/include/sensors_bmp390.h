// BMP390 sensor task API
#pragma once

#include <Arduino.h>

typedef struct {
  double temperature_c; // Celsius
  double pressure_pa;   // Pascals
  double altitude_m;    // Meters
  bool   valid;         // True if last read succeeded
} bmp_reading_t;

void bmp390_start_task();
bool bmp390_get(bmp_reading_t &out);

