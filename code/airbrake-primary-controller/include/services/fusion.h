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
  // Kinematics
  float vz_mps;         // vertical speed from AGL derivative
  float vz_acc_mps;     // vertical speed from accel integration (experimental)
  float vz_fused_mps;   // fused vertical speed (baro+acc)
  float az_imu1_mps2;   // vertical accel from IMU1 (earth frame)
  // Atmospherics
  float temp_c;         // BMP1 temperature
  float press_hPa;      // BMP1 pressure (hPa)
  float sos_mps;        // speed of sound from temperature
  float mach_vz;        // |vz| / sos
  // Attitude
  float yaw_deg, pitch_deg, roll_deg; // from IMU1 quaternion
  float tilt_deg;       // angle between +Xbody (nose) and Earth +Z (Up)
  float tilt_az_deg;    // azimuth of tilt direction around Earth +Z (atan2(y,x))
  // Predictive
  float t_apogee_s;     // biased early
  float apogee_agl_m;   // biased low
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
