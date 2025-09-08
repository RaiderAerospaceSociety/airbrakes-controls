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

// Task stack sizes and priorities
#ifndef TASK_STACK_BMP390
#define TASK_STACK_BMP390 4096
#endif
#ifndef TASK_STACK_LOGGER
#define TASK_STACK_LOGGER 3072
#endif
#ifndef TASK_STACK_LED
#define TASK_STACK_LED 2048
#endif

#ifndef TASK_PRIO_BMP390
#define TASK_PRIO_BMP390 3
#endif
#ifndef TASK_PRIO_LOGGER
#define TASK_PRIO_LOGGER 1
#endif
#ifndef TASK_PRIO_LED
#define TASK_PRIO_LED 1
#endif

// Periods
#ifndef BMP390_PERIOD_MS
#define BMP390_PERIOD_MS 100
#endif
#ifndef LOGGER_PERIOD_MS
#define LOGGER_PERIOD_MS 50
#endif
#ifndef LED_PERIOD_MS
#define LED_PERIOD_MS 15
#endif
// Default LED mode (see include/task_led.h)
#ifndef LED_MODE_DEFAULT
#define LED_MODE_DEFAULT 0  // 0=STATUS, 1=SENSORS, 2=TILT
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
#define FUSION_W_BMP1 0.70f        // weight for BMP1 in fused AGL
#endif
#ifndef FUSION_VZ_ALPHA
#define FUSION_VZ_ALPHA 0.85f      // smoothing for vertical speed derivative (0..1)
#endif
#ifndef FUSION_VZ_MAX_DT_MS
#define FUSION_VZ_MAX_DT_MS 200    // cap dt to avoid spikes on first tick
#endif
#ifndef FUSION_SAFE_TAPX_FACTOR
#define FUSION_SAFE_TAPX_FACTOR 0.7f  // bias to predict apogee earlier (<=1)
#endif
#ifndef FUSION_SAFE_ZAPX_FACTOR
#define FUSION_SAFE_ZAPX_FACTOR 0.8f  // bias to under-estimate apogee altitude (<=1)
#endif
#ifndef FUSION_USE_ACC_INT
#define FUSION_USE_ACC_INT 1       // compute experimental vz from accel integration
#endif
#ifndef FUSION_VZ_FUSE_BETA
#define FUSION_VZ_FUSE_BETA 0.2f   // fused vz = beta*baro + (1-beta)*acc (favor IMU1)
#endif
// Tilt azimuth smoothing (unit-vector EMA) and validity threshold
#ifndef FUSION_TILT_AZ_ALPHA
#define FUSION_TILT_AZ_ALPHA 0.9f   // 0..1, higher = more smoothing
#endif
#ifndef FUSION_TILT_AZ_MIN_TILT_DEG
#define FUSION_TILT_AZ_MIN_TILT_DEG 2.0f // require at least this tilt to update azimuth
#endif

// Conservative Mach gating helpers ------------------------------------------
#ifndef TILT_MAX_DEPLOY_DEG
#define TILT_MAX_DEPLOY_DEG 20.0f   // worst-case tilt used for Mach along body proxy
#endif
#ifndef SOS_10KFT_DELTA_K
#define SOS_10KFT_DELTA_K 19.8f     // ~6.5 K/km * 3.048 km; temp drop to 10k ft
#endif
#ifndef SOS_MIN_FLOOR_MPS
#define SOS_MIN_FLOOR_MPS 300.0f    // absolute floor for conservative SoS (very cold)
#endif

// Telemetry & SD logging
#ifndef USFS_PERIOD_MS
#define USFS_PERIOD_MS 20
#endif
// IMU2 (MPU6050) poll period
#ifndef IMU2_PERIOD_MS
#define IMU2_PERIOD_MS 20
#endif
// DRDY is not used; USFS is polled at USFS_PERIOD_MS
#ifndef TELEM_PERIOD_MS
#define TELEM_PERIOD_MS 20
#endif

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
#define SD_PROBE_ON_BOOT 1   // quick one-time SD wiring probe during setup()
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
// Moved to dedicated monitor config for clarity
#include "config/monitor_config.h"
#ifndef MON_ENABLE_TIME_MS
#define MON_ENABLE_TIME_MS 1
#endif
#ifndef MON_ENABLE_BMP1
#define MON_ENABLE_BMP1 0
#endif
#ifndef MON_ENABLE_IMU1_YPR
#define MON_ENABLE_IMU1_YPR 0
#endif
#ifndef MON_ENABLE_IMU1_ACCEL
#define MON_ENABLE_IMU1_ACCEL 1
#endif
#ifndef MON_ENABLE_IMU2_ACCEL
#define MON_ENABLE_IMU2_ACCEL 1
#endif
#ifndef MON_ENABLE_IMU2_GYRO
#define MON_ENABLE_IMU2_GYRO 1
#endif
#ifndef MON_ENABLE_IMU2_TEMP
#define MON_ENABLE_IMU2_TEMP 0
#endif
#ifndef MON_PRINT_HEADER_EVERY
#define MON_PRINT_HEADER_EVERY 50
#endif

// SECTION - LED/Pixel Config -------------------------------------------------
// Visual boot animation and steady run color
#ifndef LED_BOOT_STEPS
#define LED_BOOT_STEPS 128      // Number of color steps in boot sequence
#endif
#ifndef LED_BOOT_DELAY_MS
#define LED_BOOT_DELAY_MS 8     // Delay per step (ms)
#endif
#ifndef LED_RUN_COLOR
#define LED_RUN_COLOR 0x00FF00  // Solid green while running
#endif
// !SECTION

// Sensor config (sea level pressure, IMU orientation, etc.)
#include "config/sensors_config.h"

// Default app core for tasks (ESP32-S3: 1 is App core)
#ifndef APP_CPU_NUM
#define APP_CPU_NUM 1
#endif
// Core to run the SD logging task on (0=PRO CPU, 1=APP CPU)
#ifndef SD_TASK_CORE
#define SD_TASK_CORE 0
#endif
// !SECTION

// Optional: override values for bench testing (compile with -D DESK_MODE=1)
#if defined(DESK_MODE) && DESK_MODE
#include "config/desk_mode.h"
#endif
