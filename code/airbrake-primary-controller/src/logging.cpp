// Logging globals
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "logging.h"

SemaphoreHandle_t g_log_mutex = nullptr;

