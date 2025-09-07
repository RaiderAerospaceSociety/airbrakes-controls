// ===== Telemetry Aggregator =====
// Brief: Builds periodic snapshot for monitoring and (optional) SD logging.
// Refs: docs/telemetry.md, docs/signals.md
//* -- Includes --
// Telemetry aggregator and (optional) SD logger queue
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "app_config.h"
#include "telemetry.h"
#include "sensor_bmp390.h"
#include "sensor_imu1.h"
#include "sensor_imu2.h"
#include "logging.h"
#include "services/fc.h"
// !SECTION

#if LOG_BINARY_ON_SD
#include <SPI.h>
#include <SD.h>
#endif

//* -- Module Globals --
static SemaphoreHandle_t s_telem_mutex = nullptr;
static TelemetryRecord   s_latest = {};
//

#if LOG_BINARY_ON_SD
static QueueHandle_t     s_telem_q = nullptr;
static File              s_log_file;
#endif

//* -- Helpers --
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
  crc ^= 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    uint8_t byte = data[i];
    crc ^= byte;
    for (uint8_t j = 0; j < 8; j++) {
      uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

//* -- Build Snapshot --
static void telemetry_build(TelemetryRecord &rec, uint32_t seq) {
  memset(&rec, 0, sizeof(rec));
  rec.hdr.magic0 = 0xAB;
  rec.hdr.magic1 = 0xCD;
  rec.hdr.version = TELEM_VERSION;
  rec.hdr.packet_type = 0;
  rec.hdr.seq = seq;
  rec.hdr.timestamp_ms = millis();
  rec.hdr.present_flags = TP_BMP390 | TP_IMU1 | TP_IMU2 | TP_SYSTEM | TP_CONTROL;

  bmp_reading_t bmp;
  if (bmp390Get(bmp) && bmp.valid) {
    rec.bmp390.temperature_c = bmp.temperature_c;
    rec.bmp390.pressure_pa   = bmp.pressure_pa;
    rec.bmp390.altitude_m    = bmp.altitude_m;
    rec.bmp390.status        = 0;
  }

  imu1_reading_t u1;
  if (imu1Get(u1) && u1.valid) {
    rec.imu1.quat[0] = u1.quat[0];
    rec.imu1.quat[1] = u1.quat[1];
    rec.imu1.quat[2] = u1.quat[2];
    rec.imu1.quat[3] = u1.quat[3];
    rec.imu1.cal_status = 0;
    rec.imu1.dhi_rsq = 0.0f;
  }

  imu2_reading_t u2;
  if (imu2Get(u2) && u2.valid) {
    rec.imu2.accel_g[0] = u2.accel_g[0];
    rec.imu2.accel_g[1] = u2.accel_g[1];
    rec.imu2.accel_g[2] = u2.accel_g[2];
    rec.imu2.gyro_dps[0] = u2.gyro_dps[0];
    rec.imu2.gyro_dps[1] = u2.gyro_dps[1];
    rec.imu2.gyro_dps[2] = u2.gyro_dps[2];
    rec.imu2.temp_c = u2.temp_c;
    rec.imu2.status = 0;
  }

  rec.sys.vbat_mv = 0; // TODO: wire ADC later
  rec.sys.i2c_errs = 0;
  rec.sys.spi_errs = 0;
  {
    svc::FcStatus st;
    if (svc::fcGetStatus(st)) {
      rec.sys.fc_state = st.state;
      rec.sys.fc_flags = st.flags;
      rec.ctl.airbrake_cmd_deg = st.airbrake_cmd_deg;
    } else {
      rec.sys.fc_state = 0;
      rec.sys.fc_flags = 0;
      rec.ctl.airbrake_cmd_deg = 0.0f;
    }
  }
  rec.sys.cpu_temp_c = 0.0f;

  rec.ctl.airbrake_actual_deg = 0.0f;

#if LOG_INCLUDE_CRC
  rec.crc32 = crc32_update(0, reinterpret_cast<const uint8_t*>(&rec), sizeof(rec) - sizeof(rec.crc32));
#else
  rec.crc32 = 0;
#endif
}

//* -- Tasks --
static void task_telem_agg(void *param) {
  if (!s_telem_mutex) s_telem_mutex = xSemaphoreCreateMutex();
  uint32_t seq = 0;
  const TickType_t period = pdMS_TO_TICKS(TELEM_PERIOD_MS);
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    TelemetryRecord rec;
    telemetry_build(rec, seq++);
    if (s_telem_mutex) { xSemaphoreTake(s_telem_mutex, portMAX_DELAY); s_latest = rec; xSemaphoreGive(s_telem_mutex); }
#if LOG_BINARY_ON_SD
    if (s_telem_q) xQueueSend(s_telem_q, &rec, 0);
#endif
    vTaskDelayUntil(&last, period);
  }
}

#if LOG_BINARY_ON_SD
static void task_sd_writer(void *param) {
  const uint8_t cs = PIN_CS_SD1; // from pins.h
  if (g_spi_mutex) xSemaphoreTake(g_spi_mutex, portMAX_DELAY);
  bool ok = SD.begin(cs);
  if (g_spi_mutex) xSemaphoreGive(g_spi_mutex);
  if (!ok) {
    LOGLN("SD init failed; disabling SD logging");
    vTaskDelete(NULL);
    return;
  }
  if (g_spi_mutex) xSemaphoreTake(g_spi_mutex, portMAX_DELAY);
  s_log_file = SD.open("log.bin", FILE_WRITE);
  if (g_spi_mutex) xSemaphoreGive(g_spi_mutex);
  if (!s_log_file) {
    LOGLN("SD open failed: log.bin");
    vTaskDelete(NULL);
    return;
  }
  TelemetryRecord batch[LOG_BATCH_MAX_RECORDS];
  for (;;) {
    size_t n = 0;
    uint32_t t0 = millis();
    // Always get at least one if available
    if (xQueueReceive(s_telem_q, &batch[n], pdMS_TO_TICKS(LOG_BATCH_MAX_MS)) == pdTRUE) {
      n++;
    }
    // Drain remaining until either batch full or timeout reached
    while (n < LOG_BATCH_MAX_RECORDS && (millis() - t0) < LOG_BATCH_MAX_MS) {
      TelemetryRecord rec;
      if (xQueueReceive(s_telem_q, &rec, 0) == pdTRUE) {
        batch[n++] = rec;
      } else {
        break;
      }
    }
    if (n > 0) {
      if (g_spi_mutex) xSemaphoreTake(g_spi_mutex, portMAX_DELAY);
      s_log_file.write(reinterpret_cast<uint8_t*>(batch), n * sizeof(TelemetryRecord));
      s_log_file.flush();
      if (g_spi_mutex) xSemaphoreGive(g_spi_mutex);
    }
  }
}
#endif
// !SECTION

//* -- API --
bool telemetryGetLatest(TelemetryRecord &out) {
  if (s_telem_mutex) xSemaphoreTake(s_telem_mutex, portMAX_DELAY);
  out = s_latest;
  if (s_telem_mutex) xSemaphoreGive(s_telem_mutex);
  return true;
}

extern "C" void telemetryStartTasks() {
  xTaskCreatePinnedToCore(task_telem_agg, "telem", 4096, nullptr, TASK_PRIO_LOGGER, nullptr, APP_CPU_NUM);
#if LOG_BINARY_ON_SD
  if (!s_telem_q) s_telem_q = xQueueCreate(128, sizeof(TelemetryRecord));
  xTaskCreatePinnedToCore(task_sd_writer, "sdlog", 4096, nullptr, TASK_PRIO_LOGGER, nullptr, SD_TASK_CORE);
#endif
}
