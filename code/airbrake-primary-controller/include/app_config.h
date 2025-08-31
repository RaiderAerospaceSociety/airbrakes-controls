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

// Telemetry & SD logging
#ifndef USFS_PERIOD_MS
#define USFS_PERIOD_MS 20
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
