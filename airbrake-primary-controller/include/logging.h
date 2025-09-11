// ===== Logging API =====
// Brief: Mutex-guarded Serial logging macros for thread-safe prints.
//* -- Overview --
#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "app_config.h"

/** @brief Global logging mutex used to guard Serial prints. */
extern SemaphoreHandle_t g_log_mutex;

/** @brief Initialize the logging mutex (idempotent). */
inline void logging_setup_mutex()
{
  if (!g_log_mutex)
    g_log_mutex = xSemaphoreCreateMutex();
}

/** @brief Thread-safe formatted print to Serial (always prints). */
#define LOGF(fmt, ...)                            \
  do                                              \
  {                                               \
    if (g_log_mutex)                              \
      xSemaphoreTake(g_log_mutex, portMAX_DELAY); \
    Serial.printf((fmt), ##__VA_ARGS__);          \
    if (g_log_mutex)                              \
      xSemaphoreGive(g_log_mutex);                \
  } while (0)

/** @brief Thread-safe println to Serial (always prints). */
#define LOGLN(msg)                                \
  do                                              \
  {                                               \
    if (g_log_mutex)                              \
      xSemaphoreTake(g_log_mutex, portMAX_DELAY); \
    Serial.println((msg));                        \
    if (g_log_mutex)                              \
      xSemaphoreGive(g_log_mutex);                \
  } while (0)

/** @brief Thread-safe formatted print to Serial when DEBUG_ENABLED==1. */
#define DEBUGF(fmt, ...)                            \
  do                                                \
  {                                                 \
    if (DEBUG_ENABLED)                              \
    {                                               \
      if (g_log_mutex)                              \
        xSemaphoreTake(g_log_mutex, portMAX_DELAY); \
      Serial.printf((fmt), ##__VA_ARGS__);          \
      if (g_log_mutex)                              \
        xSemaphoreGive(g_log_mutex);                \
    }                                               \
  } while (0)

/** @brief Thread-safe println to Serial when DEBUG_ENABLED==1. */
#define DEBUGLN(msg)                                \
  do                                                \
  {                                                 \
    if (DEBUG_ENABLED)                              \
    {                                               \
      if (g_log_mutex)                              \
        xSemaphoreTake(g_log_mutex, portMAX_DELAY); \
      Serial.println((msg));                        \
      if (g_log_mutex)                              \
        xSemaphoreGive(g_log_mutex);                \
    }                                               \
  } while (0)

// !SECTION
