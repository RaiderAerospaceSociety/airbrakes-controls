// Telemetry record types and helpers
#pragma once

#include <stdint.h>

#define TELEM_VERSION 1

// Section presence bitmask (reserved for future dynamic enabling)
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
struct TelemetryHeader {
  uint8_t  magic0;         // 0xAB
  uint8_t  magic1;         // 0xCD
  uint8_t  version;        // TELEM_VERSION
  uint8_t  packet_type;    // 0 = full record
  uint32_t seq;            // monotonically increasing
  uint32_t timestamp_ms;   // millis()
  uint32_t present_flags;  // TP_* bitmask
};

struct TelemetryBmp390 {
  float    temperature_c;  // C
  float    pressure_pa;    // Pa
  float    altitude_m;     // meters
  uint8_t  status;         // bitfield (0 ok)
  uint8_t  _pad[3];
};

struct TelemetryImu1 {
  float    quat[4];        // w,x,y,z
  uint8_t  cal_status;     // 0 if unknown
  uint8_t  _pad[3];
  float    dhi_rsq;        // 0.0 if unused
};
// Backward-compat alias
using TelemetryUsfsmax = TelemetryImu1;

struct TelemetryImu2 {
  float    accel_g[3];     // ax,ay,az in g
  float    gyro_dps[3];    // gx,gy,gz in deg/s
  float    temp_c;         // temperature C
  uint8_t  status;         // 0 if ok
  uint8_t  _pad[3];
};

struct TelemetrySystem {
  uint16_t vbat_mv;        // millivolts
  uint16_t i2c_errs;       // counters
  uint16_t spi_errs;
  uint8_t  fc_state;       // airbrake FSM state
  uint8_t  _pad0;
  float    cpu_temp_c;     // optional
};

struct TelemetryControl {
  float    airbrake_cmd_deg;
  float    airbrake_actual_deg;
};


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
// Copy the most recent telemetry snapshot into `out`.
// Returns true if a snapshot was available.
bool telemetry_get_latest(TelemetryRecord &out);

// Start telemetry-related FreeRTOS tasks (aggregator and optional SD logger).
extern "C" void telemetry_start_tasks();
#endif
