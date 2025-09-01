// Health/FDI (fault detection and isolation) service API (scaffold)
#pragma once

#include <Arduino.h>

namespace svc {

struct HealthResiduals {
  float imu_accel_diff_g[3]; // imu1 - imu2 per axis
  float imu_gyro_diff_dps[3];
  float altitude_diff_m;     // example: bmp390 vs usfsmax if present
};

void health_init();
void health_update(); // future: compute residuals and flags
bool health_get(HealthResiduals &out); // snapshot of latest residuals

} // namespace svc

