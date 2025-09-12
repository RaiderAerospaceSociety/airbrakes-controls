// Telemetry record types and helpers
#pragma once

#include <stdint.h>

#define TELEM_VERSION 3
// TODO(telemetry, versioning): Bump TELEM_VERSION when field layout changes.
// Checklist for bumps:
//  - Update docs/telemetry.md and docs/signals.md
//  - Note changes in commit message and PR description
//  - Coordinate consumers (downstream tools/parsers)

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
  uint8_t magic0;         ///< 0xAB
  uint8_t magic1;         ///< 0xCD
  uint8_t version;        ///< TELEM_VERSION
  uint8_t packet_type;    ///< 0 = full record
  uint32_t seq;           ///< Monotonically increasing sequence
  uint32_t timestamp_ms;  ///< millis()
  uint32_t present_flags; ///< TP_* bitmask
};

/** @brief Telemetry section for BMP390. */
struct TelemetryBmp1
{
  float temperature_c; ///< Temperature (C)
  float pressure_pa;   ///< Pressure (Pa)
  float altitude_m;    ///< Altitude (m)
  uint8_t status;      ///< Bitfield (0 = ok)
  uint8_t _pad[3];
};

/** @brief Telemetry section for IMU1 (USFSMAX). */
struct TelemetryImu1
{
  float quat[4];      ///< Quaternion w,x,y,z
  uint8_t cal_status; ///< 0 if unknown
  uint8_t _pad[3];
  float dhi_rsq; ///< 0.0 if unused
};
// Backward-compat alias
using TelemetryUsfsmax = TelemetryImu1;

/** @brief Telemetry section for IMU2 (MPU6050). */
struct TelemetryImu2
{
  float accel_g[3];  ///< Accel (g)
  float gyro_dps[3]; ///< Gyro (deg/s)
  float temp_c;      ///< Temperature (C)
  uint8_t status;    ///< 0 if ok
  uint8_t _pad[3];
};

/** @brief System status metrics. */
struct TelemetrySystem
{
  uint16_t vbat_mv;  ///< Battery voltage (mV)
  uint16_t i2c_errs; ///< I2C error counter
  uint16_t spi_errs; ///< SPI error counter
  uint8_t fc_state;  ///< Airbrake FSM state
  uint8_t _pad0;
  float cpu_temp_c;  ///< CPU temperature (C)
  uint32_t fc_flags; ///< Controller flags (bitmask)
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
