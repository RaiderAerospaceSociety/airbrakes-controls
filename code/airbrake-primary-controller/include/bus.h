// ANCHOR: Overview
// SECTION - Bus API ----------------------------------------------------------
// Shared bus setup and mutexes
#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "pins.h"

extern SemaphoreHandle_t g_spi_mutex;
extern SemaphoreHandle_t g_i2c_mutex;

inline void bus_setup() {
  // Create mutexes
  if (!g_spi_mutex) g_spi_mutex = xSemaphoreCreateMutex();
  if (!g_i2c_mutex) g_i2c_mutex = xSemaphoreCreateMutex();

  // I2C once for all devices
  Wire.begin(PIN_SDA1, PIN_SCL1);

  // SPI once for all devices (CS is per device)
  SPI.end();
  SPI.begin(PIN_SCK1, PIN_MISO1, PIN_MOSI1, PIN_CS_BMP1);
}

// Scan the I2C bus and print discovered device addresses (debug-friendly)
void bus_scan_i2c();
// !SECTION
