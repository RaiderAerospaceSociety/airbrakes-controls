// ANCHOR: Overview
// SECTION - App Configuration ------------------------------------------------
// Application configuration and tuning
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
#define LOG_BINARY_ON_SD 0
#endif
#ifndef LOG_INCLUDE_CRC
#define LOG_INCLUDE_CRC 0
#endif
#ifndef LOG_INCLUDE_QUAT
#define LOG_INCLUDE_QUAT 1
#endif

// SECTION - Serial Monitor Output -------------------------------------------
// Choose which values to print in logger CSV output
#ifndef MON_LOG_FROM_TELEM
#define MON_LOG_FROM_TELEM 1  // Prefer telemetry snapshot when available
#endif
#ifndef SERIAL_PLOTTER_MODE
#define SERIAL_PLOTTER_MODE 1         // 1 = VSCode Serial Plotter format, 0 = CSV
#endif

// Enable side-by-side accelerometer comparison output
#ifndef PLOT_COMPARE_ACCEL
#define PLOT_COMPARE_ACCEL 1          // 1 = emit imu1 vs imu2 accel pairs
#endif

// Plotter single-channel selection (only used when SERIAL_PLOTTER_MODE=1)
#define PLOT_SRC_IMU1_AX  1
#define PLOT_SRC_IMU1_AY  2
#define PLOT_SRC_IMU1_AZ  3
#define PLOT_SRC_IMU2_AX  4
#define PLOT_SRC_IMU2_AY  5
#define PLOT_SRC_IMU2_AZ  6
#define PLOT_SRC_IMU2_GX  7
#define PLOT_SRC_IMU2_GY  8
#define PLOT_SRC_IMU2_GZ  9

#ifndef PLOT_SOURCE
#define PLOT_SOURCE PLOT_SRC_IMU1_AX  // default: IMU1 accel X (g)
#endif
#ifndef PLOT_VAR_LABEL
#define PLOT_VAR_LABEL "imu1_ax_g"    // label used by Serial Plotter
#endif

// Axis mask for comparison mode (bit0=X, bit1=Y, bit2=Z)
#ifndef PLOT_ACCEL_AXES_MASK
#define PLOT_ACCEL_AXES_MASK 0x1      // default: X only; set 0x7 for XYZ
#endif
#ifndef PLOT_INCLUDE_DIFF
#define PLOT_INCLUDE_DIFF 1           // include diff (imu1 - imu2) per axis
#endif
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

// Sea level pressure for altitude calc
#ifndef SEALEVELPRESSURE_HPA
#define SEALEVELPRESSURE_HPA (1012.0)
#endif

// Default app core for tasks (ESP32-S3: 1 is App core)
#ifndef APP_CPU_NUM
#define APP_CPU_NUM 1
#endif
// !SECTION
