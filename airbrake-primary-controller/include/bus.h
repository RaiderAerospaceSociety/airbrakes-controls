// ===== Bus API =====
// Brief: Shared bus setup and mutexes for I2C/SPI operations.
//* -- Overview --
#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "pins.h"

/** @brief Global SPI bus mutex (protects SPI transactions). */
extern SemaphoreHandle_t g_spi_mutex;
/** @brief Global I2C bus mutex (protects Wire transactions). */
extern SemaphoreHandle_t g_i2c_mutex;
/** @brief Global setup mutex (protects device setup). */
extern SemaphoreHandle_t g_setup_mutex;

/** @brief Initialize I2C and SPI buses and create mutexes (idempotent). */
inline void bus_setup()
{
  // Create mutexes
  if (!g_spi_mutex)
    g_spi_mutex = xSemaphoreCreateMutex();
  if (!g_i2c_mutex)
    g_i2c_mutex = xSemaphoreCreateMutex();
  if (!g_setup_mutex)
    g_setup_mutex = xSemaphoreCreateMutex();

  // I2C once for all devices
  Wire.begin(PIN_SDA1, PIN_SCL1);

  // SPI once for all devices (CS is per device)
  SPI.end();
  SPI.begin(PIN_SCK1, PIN_MISO1, PIN_MOSI1, PIN_CS_BMP1);
}

/** @brief Scan the I2C bus and print discovered addresses (debug-friendly). */
void bus_scan_i2c();

/** @brief Probe SD wiring on SPI and print diagnostics. */
void bus_probe_sd();
// !SECTION
