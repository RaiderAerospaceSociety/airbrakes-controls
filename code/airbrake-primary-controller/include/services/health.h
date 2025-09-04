// Health/FDI (fault detection and isolation) service API (scaffold)
#pragma once

#include <Arduino.h>

namespace svc {

/** @brief Residuals used for health/fault detection (scaffold).
 *  @note Units/frames documented per field.
 */
struct HealthResiduals {
  float imu_accel_diff_g[3]; ///< IMU1 - IMU2 accel (g), per axis (body frame)
  float imu_gyro_diff_dps[3];///< IMU1 - IMU2 gyro (deg/s), per axis (body frame)
  float altitude_diff_m;     ///< Altitude difference (m), e.g., BMP390 vs IMU1 baro
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

} // namespace svc
