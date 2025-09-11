// Task configuration (stacks, priorities, periods, core selection)
#pragma once

// Stacks
#ifndef TASK_STACK_BMP390
#define TASK_STACK_BMP390 4096
#endif
#ifndef TASK_STACK_LOGGER
#define TASK_STACK_LOGGER 3072
#endif
#ifndef TASK_STACK_LED
#define TASK_STACK_LED 2048
#endif

// Priorities
#ifndef TASK_PRIO_BMP390
#define TASK_PRIO_BMP390 3
#endif
#ifndef TASK_PRIO_LOGGER
#define TASK_PRIO_LOGGER 1
#endif
#ifndef TASK_PRIO_LED
#define TASK_PRIO_LED 1
#endif

// Periods (ms)
#ifndef BMP390_PERIOD_MS
#define BMP390_PERIOD_MS 100
#endif
#ifndef LOGGER_PERIOD_MS
#define LOGGER_PERIOD_MS 50
#endif
#ifndef LED_PERIOD_MS
#define LED_PERIOD_MS 15
#endif
#ifndef USFS_PERIOD_MS
#define USFS_PERIOD_MS 20
#endif
#ifndef IMU2_PERIOD_MS
#define IMU2_PERIOD_MS 20
#endif
#ifndef TELEM_PERIOD_MS
#define TELEM_PERIOD_MS 20
#endif

// Core selection
#ifndef APP_CPU_NUM
#define APP_CPU_NUM 1
#endif
#ifndef SD_TASK_CORE
#define SD_TASK_CORE 0
#endif
