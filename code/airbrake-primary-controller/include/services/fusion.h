// Fusion / Derivation service API
#pragma once

#include <Arduino.h>

namespace svc {

// Optional IMU fusion placeholder (unused for now)
struct FusedImu {
  float quat[4];
  float accel_g[3];
  float gyro_dps[3];
  float quality; // 0..1
};

// Altitude-related derived values
struct FusedAlt {
  float bmp1_alt_m;     // raw altitude from BMP1
  float imu1_alt_m;     // raw altitude from IMU1 internal baro
  float agl_bmp1_m;     // AGL from BMP1
  float agl_imu1_m;     // AGL from IMU1
  float agl_fused_m;    // fused AGL
  bool  agl_ready;      // baseline captured
};

// Start background fusion task
void fusion_start_task();

// Snapshot latest fused/derived altitude values
bool fusion_get_alt(FusedAlt &out);

// Legacy placeholders
void fusion_init();
void fusion_update();
bool fusion_get(FusedImu &out);

} // namespace svc
