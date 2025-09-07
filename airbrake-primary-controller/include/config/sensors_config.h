// Sensor configuration (ranges, I2C speeds, calibration toggles)
// NOTE: Scaffold only — current values come from app_config.h and device modules
#pragma once

// Future home for:
// - I2C clock config per device
// - Default IMU ranges and LPF
// - Sea level pressure default for altitude
// Default sea level pressure for altitude calculations (hPa)
#ifndef SEALEVELPRESSURE_HPA
#define SEALEVELPRESSURE_HPA (1012.0)
#endif

// IMU2 (MPU6050) orientation mapping to rocket body frame
// Define a 3x3 rotation matrix R such that: v_body = R * v_sensor
// Default is identity (assumes sensor axes already aligned to body axes)
#ifndef IMU2_R00
#define IMU2_R00 1.0f
#define IMU2_R01 0.0f
#define IMU2_R02 0.0f
#define IMU2_R10 0.0f
#define IMU2_R11 1.0f
#define IMU2_R12 0.0f
#define IMU2_R20 0.0f
#define IMU2_R21 0.0f
#define IMU2_R22 1.0f
#endif
