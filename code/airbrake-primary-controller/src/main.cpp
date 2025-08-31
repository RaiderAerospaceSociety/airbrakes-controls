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
#include "sensors_bmp390.h"
#include "sensors_imu1.h"
#include "sensors_imu2.h"
#include "tasks_led.h"
#include "tasks_logger.h"
#include "telemetry.h"
// !SECTION

extern "C" void telemetry_start_tasks();

// SECTION - Globals ----------------------------------------------------------
// NOTE: Define the board object here so tasks can use it via board.h extern
UMS3 ums3;
// !SECTION

// SECTION - Setup ------------------------------------------------------------
// NOTE: Simple boot animation for the onboard NeoPixel, then steady green
static void pixel_boot_sequence() {
  // Ensure pixel power is on and brightness is set by caller
  // Sweep through the color wheel quickly for a brief animation
  for (int i = 0; i < LED_BOOT_STEPS; ++i) {
    ums3.setPixelColor(UMS3::colorWheel(i * (256 / LED_BOOT_STEPS)));
    delay(LED_BOOT_DELAY_MS);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

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
  pixel_boot_sequence();
  ums3.setPixelColor(LED_RUN_COLOR);

  // Start tasks
  telemetry_start_tasks();
  bmp390_start_task();
  imu1_start_task();
  imu2_start_task();
  logger_start_task();
}
// !SECTION

// SECTION - Loop -------------------------------------------------------------
void loop() {
  vTaskDelay(portMAX_DELAY);
}
// !SECTION
