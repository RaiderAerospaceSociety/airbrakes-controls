// ===== App Configuration =====
// Brief: Build-time tunables for periods, smoothing, and safety limits.
// Note: Override via PlatformIO build_flags (e.g., -D ZERO_AGL_AFTER_MS=8000).
//* -- Overview --
// Units are documented per flag.
#pragma once

// Debug toggle (can also be set via -D DEBUG_ENABLED=0/1)
#ifndef DEBUG_ENABLED
#define DEBUG_ENABLED 1
#endif

// Task configuration is centralized here
#include "config/tasks_config.h"
// Default LED mode (see include/task_led.h)
#ifndef LED_MODE_DEFAULT
#define LED_MODE_DEFAULT 0 // 0=STATUS, 1=SENSORS, 2=TILT
#endif
// Enable using the FeatherS3 blue LED for heartbeat/debug patterns
#ifndef LED_BLUE_HEARTBEAT
#define LED_BLUE_HEARTBEAT 1
#endif
// Zero AGL baseline after this many ms from boot
#ifndef ZERO_AGL_AFTER_MS
#define ZERO_AGL_AFTER_MS 10000
#endif

// Fusion/derivation tuning ---------------------------------------------------
#ifndef FUSION_W_BMP1
#define FUSION_W_BMP1 0.70f // weight for BMP1 in fused AGL
#endif
#ifndef FUSION_VZ_ALPHA
#define FUSION_VZ_ALPHA 0.85f // smoothing for vertical speed derivative (0..1)
#endif
#ifndef FUSION_VZ_MAX_DT_MS
#define FUSION_VZ_MAX_DT_MS 200 // cap dt to avoid spikes on first tick
#endif
#ifndef FUSION_SAFE_TAPX_FACTOR
#define FUSION_SAFE_TAPX_FACTOR 0.7f // bias to predict apogee earlier (<=1)
#endif
#ifndef FUSION_SAFE_ZAPX_FACTOR
#define FUSION_SAFE_ZAPX_FACTOR 0.8f // bias to under-estimate apogee altitude (<=1)
#endif
#ifndef FUSION_USE_ACC_INT
#define FUSION_USE_ACC_INT 1 // compute experimental vz from accel integration
#endif
#ifndef FUSION_VZ_FUSE_BETA
#define FUSION_VZ_FUSE_BETA 0.2f // fused vz = beta*baro + (1-beta)*acc (favor IMU1)
#endif
// Tilt azimuth smoothing (unit-vector EMA) and validity threshold
#ifndef FUSION_TILT_AZ_ALPHA
#define FUSION_TILT_AZ_ALPHA 0.9f // 0..1, higher = more smoothing
#endif
#ifndef FUSION_TILT_AZ_MIN_TILT_DEG
#define FUSION_TILT_AZ_MIN_TILT_DEG 2.0f // require at least this tilt to update azimuth
#endif

// Conservative Mach gating helpers ------------------------------------------
#ifndef TILT_MAX_DEPLOY_DEG
#define TILT_MAX_DEPLOY_DEG 20.0f // worst-case tilt used for Mach along body proxy
#endif
#ifndef SOS_10KFT_DELTA_K
#define SOS_10KFT_DELTA_K 19.8f // ~6.5 K/km * 3.048 km; temp drop to 10k ft
#endif
#ifndef SOS_MIN_FLOOR_MPS
#define SOS_MIN_FLOOR_MPS 300.0f // absolute floor for conservative SoS (very cold)
#endif

// Telemetry & SD logging

#ifndef LOG_BATCH_MAX_RECORDS
#define LOG_BATCH_MAX_RECORDS 50
#endif
#ifndef LOG_BATCH_MAX_MS
#define LOG_BATCH_MAX_MS 100
#endif
#ifndef LOG_BINARY_ON_SD
#define LOG_BINARY_ON_SD 1
#endif
#ifndef SD_PROBE_ON_BOOT
#define SD_PROBE_ON_BOOT 1 // quick one-time SD wiring probe during setup()
#endif
#ifndef SD_PROBE_WRITE_TEST
#define SD_PROBE_WRITE_TEST 0 // 1 = create/read a tiny test file during probe
#endif
#ifndef LOG_INCLUDE_CRC
#define LOG_INCLUDE_CRC 0
#endif
#ifndef LOG_INCLUDE_QUAT
#define LOG_INCLUDE_QUAT 1
#endif

// SECTION - Serial Monitor Output -------------------------------------------
// Centralized in monitor_config.h
#include "config/monitor_config.h"

// SECTION - LED/Pixel Config -------------------------------------------------
// Visual boot animation and steady run color
#ifndef LED_BOOT_STEPS
#define LED_BOOT_STEPS 128 // Number of color steps in boot sequence
#endif
#ifndef LED_BOOT_DELAY_MS
#define LED_BOOT_DELAY_MS 8 // Delay per step (ms)
#endif
#ifndef LED_RUN_COLOR
#define LED_RUN_COLOR 0x00FF00 // Solid green while running
#endif
// !SECTION

// Sensor config (sea level pressure, IMU orientation, etc.)
#include "config/sensors_config.h"
// !SECTION

// Optional: override values for bench testing (compile with -D DESK_MODE=1)
#if defined(DESK_MODE) && DESK_MODE
#include "config/desk_mode.h"
#endif
