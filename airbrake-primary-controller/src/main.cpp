// ===== Main Application =====
// Brief: Initializes board, buses, and starts all tasks.
// Refs: docs/architecture.md
//* -- Includes --
// Core
#include <Arduino.h>
#include <UMS3.h>

//* -- App Includes --
// App modules
#include "app_config.h"
#include "logging.h"
#include "bus.h"
#include "board.h"
#include "sensor_bmp390.h"
#include "sensor_imu1.h"
#include "sensor_imu2.h"
#include "task_led.h"
#include "task_logger.h"
#include "telemetry.h"
#include "services/fusion.h"
#include "services/fc.h"
#if SD_PROBE_ON_BOOT
#include <SPI.h>
#include <SD.h>
#endif
//

// Note: telemetryStartTasks() declared in telemetry.h

//* -- Globals --
// Note: Define the board object here so tasks can use it via board.h extern
UMS3 ums3;
//

//* -- Setup --
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // Init logging (mutex) and shared buses
  logging_setup_mutex();
  bus_setup();
  delay(200);
  bus_scan_i2c();

  // Board setup
  ums3.begin();
  ums3.setPixelBrightness(255 / 3);
  ums3.setPixelPower(true);
  delay(50);

  // Set initial LED to red; task_led will update as subsystems come online
  ums3.setPixelColor(0xFF0000);

#if SD_PROBE_ON_BOOT
  // Quick SD wiring probe (SPI mode) using PIN_CS_SD1
  {
    LOGLN("SD: probing...");
    if (g_spi_mutex) xSemaphoreTake(g_spi_mutex, portMAX_DELAY);
    bool sd_ok = SD.begin(PIN_CS_SD1);
    if (g_spi_mutex) xSemaphoreGive(g_spi_mutex);
    if (sd_ok) {
      // Attempt a simple filesystem op
      File root = SD.open("/");
      if (root) { LOGLN("SD: mount OK (root opened)"); root.close(); }
      else { LOGLN("SD: mount OK but root open failed"); }
    } else {
      LOGLN("SD: probe failed (check wiring/CS)");
    }
  }
#endif

  // Start tasks
  telemetryStartTasks();
  bmp390StartTask();
  imu1StartTask();
  imu2StartTask();
  svc::fusionStartTask();
  svc::fcStartTask();
  ledStartTask();
  loggerStartTask();
}
//

//* -- Loop --
void loop() {
  vTaskDelay(portMAX_DELAY);
}
//
