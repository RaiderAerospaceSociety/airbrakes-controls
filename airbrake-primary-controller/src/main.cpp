// ===== Main Application =====
// Brief: Initializes board, buses, and starts all tasks.
// Refs: docs/architecture.md
//* ===== Includes =====
// Core
#include <Arduino.h>
#include <UMS3.h>


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
// SD probe moved to bus.cpp; call via bus_probe_sd() when enabled
//* ====================


//* ===== Globals =====
// Note: Define the board object here so tasks can use it via board.h extern
UMS3 ums3;
//* ===================

//* ===== Setup =====
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  delay(1000);

  // Board setup
  ums3.begin();
  ums3.setPixelBrightness(255 / 3);
  ums3.setPixelPower(true);
  delay(50);

  // Init logging (mutex) and shared buses
  logging_setup_mutex();
  bus_setup();
  delay(200);
  bus_scan_i2c();

  // Desk Mode Alert
#if defined(DESK_MODE) && DESK_MODE
  DEBUGLN("Desk Mode: ON (scaled thresholds, reduced durations)");
#endif

  // Set initial LED to red; task_led will update as subsystems come online
  ums3.setPixelColor(0xFF0000);

#if SD_PROBE_ON_BOOT
  bus_probe_sd();
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
