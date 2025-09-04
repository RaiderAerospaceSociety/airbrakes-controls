// ===== Bus Setup & Utilities =====
// Brief: Initializes I2C/SPI buses and provides an I2C scan helper.
//* -- Includes --
#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "bus.h"
#include "logging.h"

SemaphoreHandle_t g_spi_mutex = nullptr;
SemaphoreHandle_t g_i2c_mutex = nullptr;

//* -- API --
void bus_scan_i2c() {
#if DEBUG_ENABLED
  LOGF("I2C scan on SDA:%d SCL:%d (clk=%lu Hz)\n", PIN_SDA1, PIN_SCL1, (unsigned long)Wire.getClock());
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    uint8_t err;
    if (g_i2c_mutex) xSemaphoreTake(g_i2c_mutex, portMAX_DELAY);
    Wire.beginTransmission(addr);
    err = Wire.endTransmission(true);
    if (g_i2c_mutex) xSemaphoreGive(g_i2c_mutex);
    if (err == 0) {
      LOGF(" - 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0) {
    LOGLN("I2C scan: no devices found");
  } else {
    LOGF("I2C scan: %u device(s)\n", found);
  }
#else
  (void)0; // no-op when DEBUG is disabled
#endif
}
