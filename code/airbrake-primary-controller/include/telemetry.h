// Telemetry record types and helpers
#pragma once

#include <stdint.h>

#define TELEM_VERSION 1
// TODO(telemetry, versioning): Bump TELEM_VERSION when field layout changes.
// Checklist for bumps:
//  - Update docs/telemetry.md and docs/signals.md
//  - Note changes in commit message and PR description
//  - Coordinate consumers (downstream tools/parsers)

/** @brief Section presence bitmask (reserved for future dynamic enabling). */
enum TelemetryPresent : uint32_t {
  TP_BMP390  = 1u << 0,
  TP_IMU1    = 1u << 1, // formerly USFSMAX
  TP_SYSTEM  = 1u << 2,
  TP_CONTROL = 1u << 3,
  TP_IMU2    = 1u << 4,
};
// Backward-compat alias
#ifndef TP_USFSMAX
#define TP_USFSMAX TP_IMU1
#endif

#pragma pack(push, 1)
/** @brief Telemetry header (fixed layout). */
struct TelemetryHeader {
  uint8_t  magic0;         ///< 0xAB
  uint8_t  magic1;         ///< 0xCD
  uint8_t  version;        ///< TELEM_VERSION
  uint8_t  packet_type;    ///< 0 = full record
  uint32_t seq;            ///< Monotonically increasing sequence
  uint32_t timestamp_ms;   ///< millis()
  uint32_t present_flags;  ///< TP_* bitmask
};

/** @brief Telemetry section for BMP390. */
struct TelemetryBmp390 {
  float    temperature_c;  ///< Temperature (C)
  float    pressure_pa;    ///< Pressure (Pa)
  float    altitude_m;     ///< Altitude (m)
  uint8_t  status;         ///< Bitfield (0 = ok)
  uint8_t  _pad[3];
};

/** @brief Telemetry section for IMU1 (USFSMAX). */
struct TelemetryImu1 {
  float    quat[4];        ///< Quaternion w,x,y,z
  uint8_t  cal_status;     ///< 0 if unknown
  uint8_t  _pad[3];
  float    dhi_rsq;        ///< 0.0 if unused
};
// Backward-compat alias
using TelemetryUsfsmax = TelemetryImu1;

/** @brief Telemetry section for IMU2 (MPU6050). */
struct TelemetryImu2 {
  float    accel_g[3];     ///< Accel (g)
  float    gyro_dps[3];    ///< Gyro (deg/s)
  float    temp_c;         ///< Temperature (C)
  uint8_t  status;         ///< 0 if ok
  uint8_t  _pad[3];
};

/** @brief System status metrics. */
struct TelemetrySystem {
  uint16_t vbat_mv;        ///< Battery voltage (mV)
  uint16_t i2c_errs;       ///< I2C error counter
  uint16_t spi_errs;       ///< SPI error counter
  uint8_t  fc_state;       ///< Airbrake FSM state
  uint8_t  _pad0;
  float    cpu_temp_c;     ///< CPU temperature (C)
};

/** @brief Control surfaces / actuator telemetry. */
struct TelemetryControl {
  float    airbrake_cmd_deg;    ///< Command angle (deg)
  float    airbrake_actual_deg; ///< Measured angle (deg)
};


/** @brief Full telemetry record (packed). */
struct TelemetryRecord {
  TelemetryHeader   hdr;
  TelemetryBmp390   bmp390;
  TelemetryImu1     imu1;
  TelemetryImu2     imu2;
  TelemetrySystem   sys;
  TelemetryControl  ctl;
  uint32_t          crc32; // optional; 0 if disabled
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
