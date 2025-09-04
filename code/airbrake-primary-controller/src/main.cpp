// ANCHOR: Overview
// SECTION - Core Includes ----------------------------------------------------
// Core
#include <Arduino.h>
#include <UMS3.h>

// SECTION - App Includes -----------------------------------------------------
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
// !SECTION

extern "C" void telemetry_start_tasks();

// SECTION - Globals ----------------------------------------------------------
// NOTE: Define the board object here so tasks can use it via board.h extern
UMS3 ums3;
// !SECTION

// SECTION - Setup ------------------------------------------------------------
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
  delay(100);

  // Boot-up pixel sequence then steady green while running
  ums3.setPixelColor(LED_RUN_COLOR);

  // Start tasks
  telemetry_start_tasks();
  bmp390_start_task();
  imu1_start_task();
  imu2_start_task();
  svc::fusion_start_task();
  logger_start_task();
}
// !SECTION

// SECTION - Loop -------------------------------------------------------------
void loop() {
  vTaskDelay(portMAX_DELAY);
}
// !SECTION
