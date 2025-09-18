// ===== Main Application =====
#ifndef PIN12_PROBE
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
#include "sensor_bmp1.h"
#include "sensor_imu1.h"
#include "sensor_imu2.h"
#include "task_led.h"
#include "task_monitor.h"
#include "telemetry.h"
#include "services/fusion.h"
#include "services/fc.h"
#include "actuator_servo.h"
//* ====================

//* ===== Globals =====
//? Note: Define the board object here so tasks can use it via board.h extern
UMS3 ums3;
//* ===================

//* ===== Setup =====
void setup()
{
  // Begin Serial for debug output
  Serial.begin(115200);
  // Wait for Serial
  while (!Serial)
  {
  }
  delay(2000);

  // Board setup
  ums3.begin();
  ums3.setPixelBrightness(255 / 3);
  ums3.setPixelPower(true);
  delay(50);
  DEBUGLN("===== ^ Board Initialized ^ =====\n");

  // Init logging (mutex) and shared buses
  logging_setup_mutex();
  bus_setup();
  delay(200);
  bus_scan_i2c();
  DEBUGLN("===== ^ Buses Initialized ^ =====\n");

  // Probe SD card (if enabled app_config.h)
#if SD_PROBE_ON_BOOT
  bus_probe_sd();
  DEBUGLN("===== ^ SD Probe Complete ^ =====\n");
#endif

  // Desk Mode Alert (if enabled app_config.h)
#if defined(DESK_MODE) && DESK_MODE
  DEBUGLN("Desk Mode: ON (scaled thresholds, reduced durations)");
#endif

  // Set initial LED to red; task_led will update as subsystems come online
  ums3.setPixelColor(0xFF0000);
  DEBUGLN("===== ^ Setup Complete ^ =====\n");

  // Start tasks
  telemetryStartTasks();
  bmp1StartTask();
  imu1StartTask();
  imu2StartTask();
  svc::fusionStartTask();
  svc::fcStartTask();
  servoStartTask();
  ledStartTask();
  monitorStartTask();
}
//* =================

//* ===== Loop ===== (Does nothing; all work is in tasks)
void loop()
{
  vTaskDelay(portMAX_DELAY);
}
//* ================
#endif // !PIN12_PROBE
