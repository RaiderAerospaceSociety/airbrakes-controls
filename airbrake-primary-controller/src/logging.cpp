// ===== Logging Globals =====
// Brief: Defines the global logging mutex used by LOGF/DEBUGF macros.
//* -- Includes --
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "logging.h"

//* -- Globals --
SemaphoreHandle_t g_log_mutex = nullptr;
