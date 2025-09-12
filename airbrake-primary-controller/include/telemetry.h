// Telemetry record types and helpers
#pragma once

#include <stdint.h>

// Note: Versioning is kept out of the runtime snapshot; handle in log headers if needed.

/** @brief Section presence bitmask (reserved for future dynamic enabling). */
enum TelemetryPresent : uint32_t
{
  TP_BMP390 = 1u << 0,
  TP_IMU1 = 1u << 1, // formerly USFSMAX
  TP_SYSTEM = 1u << 2,
  TP_CONTROL = 1u << 3,
  TP_IMU2 = 1u << 4,
};
// Backward-compat alias
#ifndef TP_USFSMAX
#define TP_USFSMAX TP_IMU1
#endif

#pragma pack(push, 1)
/** @brief Telemetry header (fixed layout). */
struct TelemetryHeader
{
  uint8_t  magic0;        ///< 0xAB
  uint8_t  magic1;        ///< 0xCD
  uint8_t  packet_type;   ///< 0 = full record
  uint8_t  _pad0;         ///< reserved
  uint32_t seq;           ///< Monotonically increasing sequence
  uint32_t timestamp_ms;  ///< millis()
  uint32_t present_flags; ///< TP_* bitmask
};

/** @brief Telemetry section for BMP390. */
struct TelemetryBmp1
{
  float   temperature_c; ///< Temperature (C)
  float   pressure_pa;   ///< Pressure (Pa)
  float   altitude_m;    ///< Altitude (m)
  uint8_t status;        ///< 0 = ok; nonzero = error
  uint8_t ok;            ///< convenience boolean (status == 0)
  uint8_t _pad[2];
};

/** @brief Telemetry section for IMU1 (USFSMAX). */
struct TelemetryImu1
{
  // Health
  uint8_t status;   ///< 0 = ok; nonzero = error
  uint8_t ok;       ///< convenience boolean (status == 0)
  uint8_t cal_status; ///< library calibration indicator (0 if unknown)
  uint8_t _pad0;
  // Orientation
  float quat[4];      ///< Quaternion w,x,y,z
  float euler_deg[3]; ///< Yaw,Pitch,Roll (deg)
  // Kinematics
  float accel_g[3];   ///< Accel (g)
  float gyro_dps[3];  ///< Gyro (deg/s)
  float mag_uT[3];    ///< Magnetometer (uT)
  // Baro (internal)
  float baro_alt_m;   ///< Internal baro altitude (m)
  // Cal/diagnostic
  float dhi_rsq;      ///< Hard‑iron fit quality
};
// Backward-compat alias
using TelemetryUsfsmax = TelemetryImu1;

/** @brief Telemetry section for IMU2 (MPU6050). */
struct TelemetryImu2
{
  float   accel_g[3];  ///< Accel (g)
  float   gyro_dps[3]; ///< Gyro (deg/s)
  float   temp_c;      ///< Temperature (C)
  uint8_t status;      ///< 0 = ok
  uint8_t ok;          ///< convenience boolean (status == 0)
  uint8_t _pad[2];
};

/** @brief System status metrics. */
struct TelemetrySystem
{
  uint16_t vbat_mv;   ///< Battery voltage (mV)
  uint16_t i2c_errs;  ///< I2C error counter
  uint16_t spi_errs;  ///< SPI error counter
  uint8_t  fc_state;  ///< Airbrake FSM state
  uint8_t  _pad0;
  uint32_t fc_flags;  ///< Controller flags (bitmask)
  // FC flags as explicit booleans (status lights)
  uint8_t  sens_imu1_ok;
  uint8_t  sens_bmp1_ok;
  uint8_t  sens_imu2_ok;
  uint8_t  baro_agree;
  uint8_t  mach_ok;
  uint8_t  tilt_ok;
  uint8_t  tilt_latch;
  uint8_t  liftoff_det;
  uint8_t  burnout_det;
  uint8_t  _pad1[3];
  // FC timing
  float    fc_t_since_launch_s;
  float    fc_t_to_apogee_s;
};

/** @brief Control surfaces / actuator telemetry. */
struct TelemetryControl
{
  float airbrake_cmd_deg;    ///< Command angle (deg)
  float airbrake_actual_deg; ///< Measured angle (deg)
};

/** @brief Fused/derived values snapshot (subset needed by consumers). */
struct TelemetryFused
{
  // Timing mirrors header timestamp; included for convenience if copied alone
  uint32_t stamp_ms;      ///< Snapshot time (millis)
  uint8_t  agl_ready;     ///< 1 when baseline captured and fused outputs valid
  uint8_t  _padf[3];
  // AGL and predictors
  float agl_fused_m;      ///< Fused AGL (m)
  float agl_bmp1_m;       ///< AGL from BMP1 (m)
  float agl_imu1_m;       ///< AGL from IMU1 internal baro (m)
  float t_apogee_s;       ///< Biased-early time to apogee (s)
  float apogee_agl_m;     ///< Biased-low predicted apogee AGL (m)
  // Kinematics
  float vz_mps;           ///< Vertical speed from AGL derivative (m/s)
  float vz_acc_mps;       ///< Vertical speed from accel integration (m/s)
  float vz_fused_mps;     ///< Fused vertical speed (m/s)
  float az_imu1_mps2;     ///< Vertical acceleration from IMU1 (m/s^2)
  // Attitude and gating
  float tilt_deg;         ///< Tilt angle (deg)
  float tilt_az_deg360;   ///< Tilt azimuth mapped to [0,360) deg
  float mach_cons;        ///< Conservative Mach proxy (unitless)
  float temp_c;           ///< Temperature used for fusion (C)
};

/** @brief Full telemetry record (packed). */
struct TelemetryRecord
{
  TelemetryHeader hdr;
  TelemetryBmp1 bmp390;
  TelemetryImu1 imu1;
  TelemetryImu2 imu2;
  TelemetrySystem sys;
  TelemetryControl ctl;
  TelemetryFused fused;
  uint32_t crc32; // optional; 0 if disabled
};
#pragma pack(pop)

// Telemetry APIs
#ifdef __cplusplus
/** @brief Copy the most recent telemetry snapshot into out.
 *  @return true if a snapshot was available.
 */
bool telemetryGetLatest(TelemetryRecord &out);

/** @brief Start telemetry-related FreeRTOS tasks (aggregator and optional SD logger). */
extern "C" void telemetryStartTasks();
#endif
