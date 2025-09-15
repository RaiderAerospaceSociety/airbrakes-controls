// Fusion / Derivation service API
#pragma once

#include <Arduino.h>

namespace svc
{

  // Optional IMU fusion placeholder (unused for now)
  struct FusedImu
  {
    float quat[4];
    float accel_g[3];
    float gyro_dps[3];
    float quality; // 0..1
  };

  // Altitude-related derived values
  /**
   * @brief Fused/derived snapshot for altitude/attitude and kinematics.
   * @note Frames/units:
   *  - Earth frame: ENU (+Z up) for vertical quantities.
   *  - Body frame: +X forward (nose), +Y right, +Z down for IMU quantities.
   *  - Units: meters (m), meters/second (m/s), meters/second^2 (m/s^2), degrees (deg), Pascals (Pa), hPa.
   */
  struct FusedAlt
  {
    // Snapshot timing
    uint32_t stamp_ms; ///< Snapshot time (millis)
    uint32_t age_ms;   ///< Age when read (consumer-computed; 0 here)
    float bmp1_alt_m;  ///< Raw altitude from BMP1 (m)
    float imu1_alt_m;  ///< Raw altitude from IMU1 internal baro (m)
    float agl_bmp1_m;  ///< AGL from BMP1 (m)
    float agl_imu1_m;  ///< AGL from IMU1 (m)
    float agl_fused_m; ///< Fused AGL (m)
    bool agl_ready;    ///< True after baseline captured
    // Kinematics
    float vz_mps;       ///< Vertical speed from AGL derivative (m/s)
    float vz_acc_mps;   ///< Vertical speed from accel integration (m/s)
    float vz_fused_mps; ///< Fused vertical speed (m/s)
    float az_imu1_mps2; ///< Vertical acceleration from IMU1 (m/s^2), Earth Z
    // Atmospherics
    float temp_c;         ///< Temperature (C)
    float press_hPa;      ///< Pressure (hPa)
    float sos_mps;        ///< Speed of sound from temperature (m/s)
    float mach_vz;        ///< |vz_baro| / sos (unitless)
    float sos_ground_mps; ///< SoS at ground from startup temp (m/s)
    float sos_10kft_mps;  ///< SoS estimated at +10k ft (m/s)
    float sos_min_mps;    ///< Conservative SoS used for gating (m/s)
    float mach_cons;      ///< Conservative Mach proxy (unitless)
    // Attitude
    float yaw_deg, pitch_deg, roll_deg; ///< Display Euler angles (deg)
    float tilt_deg;                     ///< Angle between +Xbody and Earth +Z (deg)
    float tilt_az_deg;                  ///< Tilt azimuth (deg, ±180) — smoothed
    float tilt_az_deg360;               ///< Tilt azimuth mapped to [0,360) deg
    float tilt_az_unwrapped_deg;        ///< Continuous tilt azimuth (deg)
    // Predictive
    float t_apogee_s;   ///< Biased-early time to apogee (s)
    float apogee_agl_m; ///< Biased-low predicted apogee AGL (m)
  };

  /** @brief Start the background fusion/derivation task. */
  void fusionStartTask();

  /** @brief Copy the latest fused/derived snapshot.
   *  @param out Filled with latest values (see units above).
   *  @return true if snapshot available.
   */
  bool fusionGetAlt(FusedAlt &out);

  /** @brief Request a soft reset of fusion internal state (baselines, filters). */
  void fusionSoftReset();

  // Legacy placeholders
  void fusion_init();
  void fusion_update();
  bool fusion_get(FusedImu &out);

} // namespace svc
