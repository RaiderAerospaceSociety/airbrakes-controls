// ANCHOR: Overview
// SECTION - Logging API ------------------------------------------------------
// Mutex-guarded Serial logging and debug macros
#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "app_config.h"

extern SemaphoreHandle_t g_log_mutex;

inline void logging_setup_mutex() {
  if (!g_log_mutex) g_log_mutex = xSemaphoreCreateMutex();
}

#define LOGF(fmt, ...)                                                                  \
  do {                                                                                  \
    if (g_log_mutex) xSemaphoreTake(g_log_mutex, portMAX_DELAY);                        \
    Serial.printf((fmt), ##__VA_ARGS__);                                                \
    if (g_log_mutex) xSemaphoreGive(g_log_mutex);                                       \
  } while (0)

#define LOGLN(msg)                                                                       \
  do {                                                                                  \
    if (g_log_mutex) xSemaphoreTake(g_log_mutex, portMAX_DELAY);                        \
    Serial.println((msg));                                                               \
    if (g_log_mutex) xSemaphoreGive(g_log_mutex);                                       \
  } while (0)

#define DEBUGF(fmt, ...)                                                                 \
  do {                                                                                  \
    if (DEBUG_ENABLED) {                                                                 \
      if (g_log_mutex) xSemaphoreTake(g_log_mutex, portMAX_DELAY);                      \
      Serial.printf((fmt), ##__VA_ARGS__);                                              \
      if (g_log_mutex) xSemaphoreGive(g_log_mutex);                                     \
    }                                                                                   \
  } while (0)

#define DEBUGLN(msg)                                                                     \
  do {                                                                                  \
    if (DEBUG_ENABLED) {                                                                 \
      if (g_log_mutex) xSemaphoreTake(g_log_mutex, portMAX_DELAY);                      \
      Serial.println((msg));                                                             \
      if (g_log_mutex) xSemaphoreGive(g_log_mutex);                                     \
    }                                                                                   \
  } while (0)

// !SECTION
