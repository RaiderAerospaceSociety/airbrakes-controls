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
