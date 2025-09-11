// Health/FDI (fault detection and isolation) service API (scaffold)
#pragma once

#include <Arduino.h>

namespace svc
{

  /** @brief Residuals used for health/fault detection (scaffold).
   *  @note Units/frames documented per field.
   */
  struct HealthResiduals
  {
    float imu_accel_diff_g[3];  ///< IMU1 - IMU2 accel (g), per axis (body frame)
    float imu_gyro_diff_dps[3]; ///< IMU1 - IMU2 gyro (deg/s), per axis (body frame)
    float altitude_diff_m;      ///< Altitude difference (m), e.g., BMP390 vs IMU1 baro
  };

  /** @brief Health flags bitmask capturing debounced gates and lockouts. */
  enum HealthFlags : uint32_t
  {
    HF_NONE = 0u,
    HF_SENS_IMU1_OK = 1u << 0,
    HF_SENS_IMU2_OK = 1u << 1,
    HF_SENS_BMP1_OK = 1u << 2,
    HF_BARO_AGREE = 1u << 3,
    HF_MACH_OK = 1u << 4,
    HF_TILT_OK = 1u << 5,
    HF_TILT_LATCH = 1u << 6, ///< Latched abort when tilt exceeded limit
  };

  /** @brief Snapshot of health state. */
  struct HealthSnapshot
  {
    uint32_t stamp_ms;         ///< millis()
    uint32_t flags;            ///< HealthFlags bitmask
    HealthResiduals residuals; ///< Optional residuals
  };

  /** @brief Initialize the health service. */
  void health_init();
  /** @brief Update residuals and flags (future). */
  void health_update();
  /** @brief Copy the latest residual snapshot.
   *  @param out Filled with latest residuals.
   *  @return true if snapshot is available.
   */
  bool health_get(HealthResiduals &out);

  /** @brief Copy the latest health flags snapshot. */
  bool health_get_flags(HealthSnapshot &out);

} // namespace svc
