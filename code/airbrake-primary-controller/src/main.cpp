// Core
#include <Arduino.h>
#include <UMS3.h>

// App modules
#include "app_config.h"
#include "logging.h"
#include "bus.h"
#include "board.h"
#include "sensors_bmp390.h"
#include "sensors_usfsmax.h"
#include "tasks_led.h"
#include "tasks_logger.h"
#include "telemetry.h"

extern "C" void telemetry_start_tasks();

// Define the board object here so tasks can use it via board.h extern
UMS3 ums3;

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

  // Start tasks
  telemetry_start_tasks();
  bmp390_start_task();
  usfsmax_start_task();
  logger_start_task();
  led_start_task();
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
