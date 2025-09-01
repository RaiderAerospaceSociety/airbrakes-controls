// Fusion / derivation service implementation
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "app_config.h"
#include "services/fusion.h"
#include "sensors_bmp390.h"
#include "sensors_imu1.h"

namespace svc {

// Weights for fused AGL (favor the more precise sensor)
#ifndef FUSION_W_BMP1
#define FUSION_W_BMP1 0.70f
#endif
#ifndef FUSION_W_IMU1
#define FUSION_W_IMU1 (1.0f - FUSION_W_BMP1)
#endif

static FusedImu s_fused_imu = {};
static FusedAlt s_fused_alt = {};
static SemaphoreHandle_t s_alt_mutex = nullptr;

// Baseline state for AGL zeroing
static bool     s_agl_ready = false;
static uint32_t s_agl_arm_ms = 0;
static float    s_base_bmp1_m = NAN;
static float    s_base_imu1_m = NAN;

static void fusion_task(void *param) {
  const TickType_t period = pdMS_TO_TICKS(TELEM_PERIOD_MS);
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    // Read raw altitudes
    bmp_reading_t b; bool vb = bmp390_get(b) && b.valid;
    imu1_reading_t u1; bool vi = imu1_get(u1) && u1.valid;
    float bmp_alt = vb ? (float)b.altitude_m : NAN;
    float imu_alt = vi ? u1.altitude_m : NAN;

    uint32_t now = millis();
    if (s_agl_arm_ms == 0) s_agl_arm_ms = now + ZERO_AGL_AFTER_MS;
    if (!s_agl_ready && now >= s_agl_arm_ms) {
      s_base_bmp1_m = bmp_alt;
      s_base_imu1_m = imu_alt;
      s_agl_ready = true;
    }

    float agl_bmp1 = NAN, agl_imu1 = NAN, agl_fused = NAN;
    if (s_agl_ready) {
      if (!isnan(s_base_bmp1_m) && !isnan(bmp_alt)) agl_bmp1 = bmp_alt - s_base_bmp1_m;
      if (!isnan(s_base_imu1_m) && !isnan(imu_alt)) agl_imu1 = imu_alt - s_base_imu1_m;
      // Weighted fusion when both available; fallback otherwise
      if (!isnan(agl_bmp1) && !isnan(agl_imu1)) {
        agl_fused = FUSION_W_BMP1 * agl_bmp1 + FUSION_W_IMU1 * agl_imu1;
      } else if (!isnan(agl_bmp1)) {
        agl_fused = agl_bmp1;
      } else if (!isnan(agl_imu1)) {
        agl_fused = agl_imu1;
      }
    }

    if (!s_alt_mutex) s_alt_mutex = xSemaphoreCreateMutex();
    if (s_alt_mutex) xSemaphoreTake(s_alt_mutex, portMAX_DELAY);
    s_fused_alt.bmp1_alt_m = bmp_alt;
    s_fused_alt.imu1_alt_m = imu_alt;
    s_fused_alt.agl_bmp1_m = agl_bmp1;
    s_fused_alt.agl_imu1_m = agl_imu1;
    s_fused_alt.agl_fused_m = agl_fused;
    s_fused_alt.agl_ready = s_agl_ready;
    if (s_alt_mutex) xSemaphoreGive(s_alt_mutex);

    vTaskDelayUntil(&last, period);
  }
}

void fusion_start_task() {
  if (!s_alt_mutex) s_alt_mutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(fusion_task, "fusion", 3072, nullptr, 1, nullptr, APP_CPU_NUM);
}

bool fusion_get_alt(FusedAlt &out) {
  if (s_alt_mutex) xSemaphoreTake(s_alt_mutex, portMAX_DELAY);
  out = s_fused_alt;
  if (s_alt_mutex) xSemaphoreGive(s_alt_mutex);
  return true;
}

// Legacy placeholders
void fusion_init() {}
void fusion_update() {}
bool fusion_get(FusedImu &out) { out = s_fused_imu; return true; }

} // namespace svc
