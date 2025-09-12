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
#include "sensor_bmp1.h"
#include "sensor_imu1.h"
#include "sensor_imu2.h"
#include "logging.h"
#include "services/fc.h"
#include "services/fusion.h"
// Buses (SPI/I2C mutexes and pin map)
#include "bus.h"
#include "board.h"
#include <USFSMAX.h>
// !SECTION

#if LOG_BINARY_ON_SD
#include <SPI.h>
#include <SD.h>
#endif

//* -- Module Globals --
static SemaphoreHandle_t s_telem_mutex = nullptr;
static TelemetryRecord s_latest = {};
//

#if LOG_BINARY_ON_SD
static QueueHandle_t s_telem_q = nullptr;
static File s_log_file;
static TelemetryRecord *s_sd_batch = nullptr; // heap buffer to avoid task stack overflow
#endif

//* -- Helpers --
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
  crc ^= 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++)
  {
    uint8_t byte = data[i];
    crc ^= byte;
    for (uint8_t j = 0; j < 8; j++)
    {
      uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

//* -- Build Snapshot --
static void telemetry_build(TelemetryRecord &rec, uint32_t seq)
{
  memset(&rec, 0, sizeof(rec));
  rec.hdr.magic0 = 0xAB;
  rec.hdr.magic1 = 0xCD;
  rec.hdr.packet_type = 0;
  rec.hdr.seq = seq;
  rec.hdr.timestamp_ms = millis();
  rec.hdr.present_flags = TP_BMP390 | TP_IMU1 | TP_IMU2 | TP_SYSTEM | TP_CONTROL;

  bmp_reading_t bmp;
  if (bmp1Get(bmp) && bmp.valid)
  {
    rec.bmp390.temperature_c = (float)bmp.temperature_c;
    rec.bmp390.pressure_pa = (float)bmp.pressure_pa;
    rec.bmp390.altitude_m = (float)bmp.altitude_m;
    rec.bmp390.status = 0;
    rec.bmp390.ok = 1;
  }
  else
  {
    rec.bmp390.status = 1;
    rec.bmp390.ok = 0;
  }

  imu1_reading_t u1;
  if (imu1Get(u1) && u1.valid)
  {
    rec.imu1.status = 0; rec.imu1.ok = 1;
    rec.imu1.quat[0] = u1.quat[0];
    rec.imu1.quat[1] = u1.quat[1];
    rec.imu1.quat[2] = u1.quat[2];
    rec.imu1.quat[3] = u1.quat[3];
    // Euler from quaternion
    auto quat_to_euler = [](float w, float x, float y, float z, float &yaw, float &pitch, float &roll)
    {
      yaw = atan2f(2.0f * (x * y + w * z), 1.0f - 2.0f * (y * y + z * z)) * 57.2957795f;
      pitch = asinf(2.0f * (w * y - z * x)) * 57.2957795f;
      roll = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * 57.2957795f;
    };
    quat_to_euler(u1.quat[0], u1.quat[1], u1.quat[2], u1.quat[3], rec.imu1.euler_deg[0], rec.imu1.euler_deg[1], rec.imu1.euler_deg[2]);
    rec.imu1.accel_g[0] = u1.accel_g[0];
    rec.imu1.accel_g[1] = u1.accel_g[1];
    rec.imu1.accel_g[2] = u1.accel_g[2];
    // Latest raw ADC samples (from USFSMAX globals)
    extern int16_t gyroADC[2][3];
    extern int16_t magADC[2][3];
    extern float UT_per_Count;
#if defined(GYRO_SCALE_2000)
    const float GYRO_DPS_PER_COUNT = 0.07f;
#elif defined(GYRO_SCALE_1000)
    const float GYRO_DPS_PER_COUNT = 0.035f;
#elif defined(GYRO_SCALE_500)
    const float GYRO_DPS_PER_COUNT = 0.0175f;
#elif defined(GYRO_SCALE_250)
    const float GYRO_DPS_PER_COUNT = 0.00875f;
#elif defined(GYRO_SCALE_125)
    const float GYRO_DPS_PER_COUNT = 0.004375f;
#else
    const float GYRO_DPS_PER_COUNT = 0.07f;
#endif
    rec.imu1.gyro_dps[0] = gyroADC[0][0] * GYRO_DPS_PER_COUNT;
    rec.imu1.gyro_dps[1] = gyroADC[0][1] * GYRO_DPS_PER_COUNT;
    rec.imu1.gyro_dps[2] = gyroADC[0][2] * GYRO_DPS_PER_COUNT;
    rec.imu1.mag_uT[0] = magADC[0][0] * UT_per_Count;
    rec.imu1.mag_uT[1] = magADC[0][1] * UT_per_Count;
    rec.imu1.mag_uT[2] = magADC[0][2] * UT_per_Count;
    rec.imu1.baro_alt_m = u1.altitude_m;
    rec.imu1.cal_status = 0;
    rec.imu1.dhi_rsq = 0.0f;
  }
  else
  {
    rec.imu1.status = 1; rec.imu1.ok = 0;
  }

  imu2_reading_t u2;
  if (imu2Get(u2) && u2.valid)
  {
    rec.imu2.accel_g[0] = u2.accel_g[0];
    rec.imu2.accel_g[1] = u2.accel_g[1];
    rec.imu2.accel_g[2] = u2.accel_g[2];
    rec.imu2.gyro_dps[0] = u2.gyro_dps[0];
    rec.imu2.gyro_dps[1] = u2.gyro_dps[1];
    rec.imu2.gyro_dps[2] = u2.gyro_dps[2];
    rec.imu2.temp_c = u2.temp_c;
    rec.imu2.status = 0;
    rec.imu2.ok = 1;
  }
  else
  {
    rec.imu2.status = 1;
    rec.imu2.ok = 0;
  }

  // System and FC flags
  rec.sys.vbat_mv = (uint16_t)(ums3.getBatteryVoltage() * 1000.0f);
  rec.sys.i2c_errs = 0;
  rec.sys.spi_errs = 0;
  {
    svc::FcStatus st;
    if (svc::fcGetStatus(st))
    {
      rec.sys.fc_state = st.state;
      rec.sys.fc_flags = st.flags;
      uint32_t ff = st.flags;
      rec.sys.sens_imu1_ok = (ff & svc::FCF_SENS_IMU1_OK) ? 1 : 0;
      rec.sys.sens_bmp1_ok = (ff & svc::FCF_SENS_BMP1_OK) ? 1 : 0;
      rec.sys.sens_imu2_ok = (ff & svc::FCF_SENS_IMU2_OK) ? 1 : 0;
      rec.sys.baro_agree   = (ff & svc::FCF_BARO_AGREE) ? 1 : 0;
      rec.sys.mach_ok      = (ff & svc::FCF_MACH_OK) ? 1 : 0;
      rec.sys.tilt_ok      = (ff & svc::FCF_TILT_OK) ? 1 : 0;
      rec.sys.tilt_latch   = (ff & svc::FCF_TILT_LATCH) ? 1 : 0;
      rec.sys.liftoff_det  = (ff & svc::FCF_LIFTOFF_DET) ? 1 : 0;
      rec.sys.burnout_det  = (ff & svc::FCF_BURNOUT_DET) ? 1 : 0;
      rec.sys.fc_t_since_launch_s = st.t_since_launch_s;
      rec.sys.fc_t_to_apogee_s    = st.t_to_apogee_s;
      rec.ctl.airbrake_cmd_deg    = st.airbrake_cmd_deg;
    }
  }

  rec.ctl.airbrake_actual_deg = 0.0f;
  // Fused/derived values snapshot
  {
    svc::FusedAlt f;
    if (svc::fusionGetAlt(f))
    {
      rec.fused.stamp_ms = rec.hdr.timestamp_ms;
      rec.fused.agl_ready = f.agl_ready ? 1 : 0;
      rec.fused.agl_fused_m = f.agl_fused_m;
      rec.fused.agl_bmp1_m = f.agl_bmp1_m;
      rec.fused.agl_imu1_m = f.agl_imu1_m;
      rec.fused.t_apogee_s = f.t_apogee_s;
      rec.fused.apogee_agl_m = f.apogee_agl_m;
      rec.fused.vz_mps = f.vz_mps;
      rec.fused.vz_acc_mps = f.vz_acc_mps;
      rec.fused.vz_fused_mps = f.vz_fused_mps;
      rec.fused.az_imu1_mps2 = f.az_imu1_mps2;
      rec.fused.tilt_deg = f.tilt_deg;
      rec.fused.tilt_az_deg360 = f.tilt_az_deg360;
      rec.fused.mach_cons = f.mach_cons;
      rec.fused.temp_c = f.temp_c;
    }
  }

#if LOG_INCLUDE_CRC
  rec.crc32 = crc32_update(0, reinterpret_cast<const uint8_t *>(&rec), sizeof(rec) - sizeof(rec.crc32));
#else
  rec.crc32 = 0;
#endif
}

//* -- Tasks --
static void task_telem_agg(void *param)
{
  if (!s_telem_mutex)
    s_telem_mutex = xSemaphoreCreateMutex();
  uint32_t seq = 0;
  TickType_t last = xTaskGetTickCount();
  for (;;)
  {
    TelemetryRecord rec;
    telemetry_build(rec, seq++);
    if (s_telem_mutex)
    {
      xSemaphoreTake(s_telem_mutex, portMAX_DELAY);
      s_latest = rec;
      xSemaphoreGive(s_telem_mutex);
    }
#if LOG_BINARY_ON_SD
    if (s_telem_q)
      xQueueSend(s_telem_q, &rec, 0);
#endif
    vTaskDelayUntil(&last, pdMS_TO_TICKS(TELEM_PERIOD_MS));
  }
}

#if LOG_BINARY_ON_SD
static void task_sd_writer(void *param)
{
  const uint8_t cs = PIN_CS_SD1; // from pins.h
  if (g_spi_mutex)
    xSemaphoreTake(g_spi_mutex, portMAX_DELAY);
  bool ok = SD.begin(cs);
  if (g_spi_mutex)
    xSemaphoreGive(g_spi_mutex);
  if (!ok)
  {
    LOGLN("SD init failed; disabling SD logging");
    vTaskDelete(NULL);
    return;
  }
  if (g_spi_mutex)
    xSemaphoreTake(g_spi_mutex, portMAX_DELAY);
  // Use absolute path. Append if exists, else create
  if (SD.exists("/log.bin"))
  {
    s_log_file = SD.open("/log.bin", FILE_APPEND);
  }
  else
  {
    s_log_file = SD.open("/log.bin", FILE_WRITE);
  }
  if (g_spi_mutex)
    xSemaphoreGive(g_spi_mutex);
  if (!s_log_file)
  {
    LOGLN("SD open failed: /log.bin");
    vTaskDelete(NULL);
    return;
  }
  // Allocate batch buffer on the heap to keep task stack small
  if (!s_sd_batch)
  {
    s_sd_batch = (TelemetryRecord *)pvPortMalloc(LOG_BATCH_MAX_RECORDS * sizeof(TelemetryRecord));
    if (!s_sd_batch)
    {
      LOGLN("SD: batch alloc failed; falling back to single-record writes");
    }
  }
  for (;;)
  {
    size_t n = 0;
    uint32_t t0 = millis();
    // Always get at least one if available
    if (s_sd_batch)
    {
      if (xQueueReceive(s_telem_q, &s_sd_batch[n], pdMS_TO_TICKS(LOG_BATCH_MAX_MS)) == pdTRUE)
      {
        n++;
      }
    }
    else
    {
      // Heap unavailable; write one-by-one
      TelemetryRecord rec;
      if (xQueueReceive(s_telem_q, &rec, pdMS_TO_TICKS(LOG_BATCH_MAX_MS)) == pdTRUE)
      {
        if (g_spi_mutex)
          xSemaphoreTake(g_spi_mutex, portMAX_DELAY);
        s_log_file.write(reinterpret_cast<uint8_t *>(&rec), sizeof(TelemetryRecord));
        s_log_file.flush();
        if (g_spi_mutex)
          xSemaphoreGive(g_spi_mutex);
        continue;
      }
    }
    // Drain remaining until either batch full or timeout reached
    if (s_sd_batch)
    {
      while (n < LOG_BATCH_MAX_RECORDS && (millis() - t0) < LOG_BATCH_MAX_MS)
      {
        TelemetryRecord rec;
        if (xQueueReceive(s_telem_q, &rec, 0) == pdTRUE)
        {
          s_sd_batch[n++] = rec;
        }
        else
        {
          break;
        }
      }
      if (n > 0)
      {
        if (g_spi_mutex)
          xSemaphoreTake(g_spi_mutex, portMAX_DELAY);
        s_log_file.write(reinterpret_cast<uint8_t *>(s_sd_batch), n * sizeof(TelemetryRecord));
        s_log_file.flush();
        if (g_spi_mutex)
          xSemaphoreGive(g_spi_mutex);
      }
    }
  }
}
#endif
// !SECTION

//* -- API --
bool telemetryGetLatest(TelemetryRecord &out)
{
  if (s_telem_mutex)
    xSemaphoreTake(s_telem_mutex, portMAX_DELAY);
  out = s_latest;
  if (s_telem_mutex)
    xSemaphoreGive(s_telem_mutex);
  return true;
}

extern "C" void telemetryStartTasks()
{
  xTaskCreatePinnedToCore(task_telem_agg, "telem", 4096, nullptr, TASK_PRIO_LOGGER, nullptr, APP_CPU_NUM);
#if LOG_BINARY_ON_SD
  if (!s_telem_q)
    s_telem_q = xQueueCreate(128, sizeof(TelemetryRecord));
  // Increase stack a bit to accommodate SD + VFS operations
  xTaskCreatePinnedToCore(task_sd_writer, "sdlog", 6144, nullptr, TASK_PRIO_LOGGER, nullptr, APP_CPU_NUM);
#endif
}
