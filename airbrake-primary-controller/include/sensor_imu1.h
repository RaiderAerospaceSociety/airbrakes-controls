// IMU1: USFSMAX (MAX32660 + MMC5983) minimal reader over I2C
#pragma once

#include <Arduino.h>

/**
 * @brief IMU1 snapshot from the USFSMAX module.
 * @note Frames and units:
 *  - quat[4]: orientation quaternion (w,x,y,z), body→earth
 *  - accel_g[3]: acceleration in body frame (g), +X nose, +Y right, +Z down
 *  - pressure_pa: internal baro pressure (Pa)
 *  - altitude_m: internal baro altitude (m), Earth frame
 */
typedef struct
{
  float quat[4];     ///< Orientation quaternion w,x,y,z (body→earth)
  float accel_g[3];  ///< Acceleration (g) in body frame
  float pressure_pa; ///< Internal baro pressure (Pa)
  float altitude_m;  ///< Internal baro altitude (m), Earth frame
  bool valid;        ///< True if data is valid
} imu1_reading_t;

/** @brief Start the IMU1 (USFSMAX) polling task. */
void imu1StartTask();

/** @brief Copy the latest IMU1 reading.
 *  @param out Filled with latest values (see units above).
 *  @return true if data is valid.
 */
bool imu1Get(imu1_reading_t &out);
