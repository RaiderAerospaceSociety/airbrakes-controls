//* ===== BMP390 Sensor Task =====
// Brief: Polls BMP390 over SPI and snapshots pressure/temperature/altitude.
// Refs: docs/sensors/bmp390.md, docs/signals.md
// BMP390 Task Implementation & API
//* ===== Includes =====
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Adafruit_BMP3XX.h>

#include "app_config.h"
#include "config/sensors_config.h"
#include "logging.h"
#include "bus.h"
#include "pins.h"
#include "sensor_bmp1.h"
#include "rtos_mutex.h"
//* ====================

//* ===== Module Globals =====
static Adafruit_BMP3XX s_bmp1;
static bool s_bmp1_ok = false;

// Data handlers
static SemaphoreHandle_t s_bmp1Data_mutex = nullptr; // protect local snapshot `s_latest`
static bmp_reading_t s_latest = {0};
//* ==========================

//* ===== Task: BMP1 (BMP390) =====
static void task_sensor_bmp1(void *param)
{
  // Enter protected setup for BMP1
  ENTER_CRITICAL(g_setup_mutex);

  // Ensure data mutex exists
  if (!s_bmp1Data_mutex)
    s_bmp1Data_mutex = xSemaphoreCreateMutex();

  // Initialize BMP1 on shared SPI
  WITH_MUTEX(g_spi_mutex)
  {
    s_bmp1_ok = s_bmp1.begin_SPI(PIN_CS_BMP1, &SPI);
  }

  // Check for BMP1 presence and kill task if not found
  if (!s_bmp1_ok)
  {
    LOGLN("BMP1 (BMP390) not found; task exiting");
    vTaskDelete(nullptr);
    return;
  }

  // Configure BMP1 (per datasheet recommendations)
  ENTER_CRITICAL(g_spi_mutex);
  s_bmp1.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  s_bmp1.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  s_bmp1.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  s_bmp1.setOutputDataRate(BMP3_ODR_50_HZ);
  uint8_t id = s_bmp1.chipID();
  EXIT_CRITICAL(g_spi_mutex);

  LOGF("BMP390 #1 initialized, chipID=0x%02X (CS=%d)\n", id, PIN_CS_BMP1);
  DEBUGLN("===== ^ BMP1 (BMP390) setup complete ^ =====\n");
  EXIT_CRITICAL(g_setup_mutex);

  TickType_t last = xTaskGetTickCount();
  for (;;)
  {
    // Perform reading and snapshot data if valid
    bool ok;
    WITH_MUTEX(g_spi_mutex)
    {
      ok = s_bmp1.performReading();
    }

    if (ok)
    {
      bmp_reading_t r;
      r.temperature_c = s_bmp1.temperature;
      r.pressure_pa = s_bmp1.pressure;
      r.altitude_m = 44330.0 * (1.0 - pow((r.pressure_pa / 100.0F) / SEALEVELPRESSURE_HPA, 0.1903));
      r.valid = true;

      WITH_MUTEX(s_bmp1Data_mutex)
      {
        s_latest = r;
      }
    }
    else
    {
      DEBUGLN("BMP390 read failed");
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(BMP390_PERIOD_MS));
  }
}
//* ======================

//* ===== Public API =====
// Initializes all BMP1 specific resources then begins the task with the given configuration
void bmp1StartTask()
{
  // Mutex for protecting s_latest
  if (!s_bmp1Data_mutex)
  {
    s_bmp1Data_mutex = xSemaphoreCreateMutex();
  }

  // Task for BMP1
  xTaskCreatePinnedToCore(task_sensor_bmp1, "bmp1", TASK_STACK_BMP390, nullptr, TASK_PRIO_BMP390, nullptr, APP_CPU_NUM);
}

// Retrieves the latest reading (snapshot) from the BMP390; returns true if valid bmp_reading_t was obtained
bool bmp1Get(bmp_reading_t &out)
{
  bool valid;
  WITH_MUTEX(s_bmp1Data_mutex)
  {
    out = s_latest;
    valid = s_latest.valid;
  }
  return valid;
}
//* ======================
