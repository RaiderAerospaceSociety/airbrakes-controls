// IMU2: MPU6050 reader over I2C (Adafruit library)
#pragma once

#include <Arduino.h>

/**
 * @brief IMU2 snapshot (MPU6050).
 * @note Frames and units:
 *  - accel_g[3]: body frame acceleration (g), +X nose, +Y right, +Z down (after mapping)
 *  - gyro_dps[3]: body frame angular rate (deg/s)
 *  - temp_c: degrees Celsius
 */
typedef struct
{
  float accel_g[3];  ///< Accel (g), body frame
  float gyro_dps[3]; ///< Gyro (deg/s), body frame
  float temp_c;      ///< Temperature (C)
  bool valid;        ///< True if the last read succeeded
} imu2_reading_t;

/** @brief Start the IMU2 (MPU6050) polling task. */
void imu2StartTask();
/** @brief Copy the latest IMU2 reading.
 *  @param out Filled with latest values (see units above).
 *  @return true if data is valid.
 */
bool imu2Get(imu2_reading_t &out);
