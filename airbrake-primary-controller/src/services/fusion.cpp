// ===== Fusion / Derivation Service =====
// Brief: Computes fused AGL, vertical speeds, tilt, atmospherics; exposes snapshot.
// Refs: docs/architecture.md, docs/signals.md
//* -- Includes --
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "app_config.h"
#include "services/fusion.h"
#include "sensor_bmp1.h"
#include "sensor_imu1.h"

namespace svc
{

// Weights for fused AGL (favor the more precise sensor)
#ifndef FUSION_W_IMU1
#define FUSION_W_IMU1 (1.0f - FUSION_W_BMP1)
#endif

  static inline void quat_to_euler(float w, float x, float y, float z, float &yaw, float &pitch, float &roll)
  {
    yaw = atan2f(2.0f * (x * y + w * z), 1.0f - 2.0f * (y * y + z * z)) * 57.2957795f;
    pitch = asinf(2.0f * (w * y - z * x)) * 57.2957795f;
    roll = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y)) * 57.2957795f;
  }

  static inline void rotate_vec_by_quat(const float q[4], const float v[3], float out[3])
  {
    // q = [w,x,y,z], v body -> earth
    float w = q[0], x = q[1], y = q[2], z = q[3];
    // Compute rotation matrix elements
    float ww = w * w, xx = x * x, yy = y * y, zz = z * z;
    float r00 = 1 - 2 * (yy + zz);
    float r01 = 2 * (x * y - w * z);
    float r02 = 2 * (x * z + w * y);
    float r10 = 2 * (x * y + w * z);
    float r11 = 1 - 2 * (xx + zz);
    float r12 = 2 * (y * z - w * x);
    float r20 = 2 * (x * z - w * y);
    float r21 = 2 * (y * z + w * x);
    float r22 = 1 - 2 * (xx + yy);
    out[0] = r00 * v[0] + r01 * v[1] + r02 * v[2];
    out[1] = r10 * v[0] + r11 * v[1] + r12 * v[2];
    out[2] = r20 * v[0] + r21 * v[1] + r22 * v[2];
  }

  static FusedImu s_fused_imu = {};
  static FusedAlt s_fused_alt = {};
  static SemaphoreHandle_t s_alt_mutex = nullptr;
  // async reset request flag
  static volatile bool s_reset_req = false;

  // Baseline state for AGL zeroing
  static bool s_agl_ready = false;
  static uint32_t s_agl_arm_ms = 0;
  static float s_base_bmp1_m = NAN;
  static float s_base_imu1_m = NAN;

  //* -- Task --
  static void fusion_task(void *param)
  {
    TickType_t last = xTaskGetTickCount();
    const float G0 = 9.80665f;
    static bool have_prev_alt = false;
    static float prev_alt = NAN;
    static uint32_t prev_ms = 0;
    static float vz_filt = NAN;
    static float vz_acc = 0.0f; // leaky integrator
    // Smoothed tilt azimuth unit vector in Earth XY
    static bool have_tilt_az = false;
    static float tiltAzX = NAN, tiltAzY = NAN;
    static bool have_tilt_az_acc = false;
    static float tilt_az_prev_deg = 0.0f;
    static float tilt_az_unwrapped = 0.0f;
    // Conservative SoS estimates (computed once when BMP valid)
    static bool have_sos_refs = false;
    static float sos_ground_mps = NAN;
    static float sos_10kft_mps = NAN;
    static float sos_min_mps = SOS_MIN_FLOOR_MPS;
    for (;;)
    {
      // Handle async soft reset requests
      if (s_reset_req)
      {
        s_reset_req = false;
        have_prev_alt = false;
        prev_alt = NAN;
        prev_ms = 0;
        vz_filt = NAN;
        vz_acc = 0.0f;
        have_tilt_az = false;
        tiltAzX = NAN; tiltAzY = NAN;
        have_tilt_az_acc = false;
        tilt_az_prev_deg = 0.0f; tilt_az_unwrapped = 0.0f;
        have_sos_refs = false;
        sos_ground_mps = NAN; sos_10kft_mps = NAN; sos_min_mps = SOS_MIN_FLOOR_MPS;
        s_agl_ready = false;
        s_agl_arm_ms = 0;
        s_base_bmp1_m = NAN;
        s_base_imu1_m = NAN;
        // Clear published snapshot
        if (!s_alt_mutex)
          s_alt_mutex = xSemaphoreCreateMutex();
        if (s_alt_mutex) xSemaphoreTake(s_alt_mutex, portMAX_DELAY);
        memset(&s_fused_alt, 0, sizeof(s_fused_alt));
        s_fused_alt.stamp_ms = millis();
        if (s_alt_mutex) xSemaphoreGive(s_alt_mutex);
      }
      // Read raw altitudes
      bmp_reading_t b;
      bool vb = bmp1Get(b) && b.valid;
      imu1_reading_t u1;
      bool vi = imu1Get(u1) && u1.valid;
      float bmp_alt = vb ? (float)b.altitude_m : NAN;
      float imu_alt = vi ? u1.altitude_m : NAN;

      uint32_t now = millis();
      if (s_agl_arm_ms == 0)
        s_agl_arm_ms = now + ZERO_AGL_AFTER_MS;
      // Arm AGL baseline after configured delay, but initialize each sensor's baseline lazily
      if (!s_agl_ready && now >= s_agl_arm_ms)
      {
        s_agl_ready = true;
      }
      if (s_agl_ready)
      {
        if (isnan(s_base_bmp1_m) && !isnan(bmp_alt))
          s_base_bmp1_m = bmp_alt;
        if (isnan(s_base_imu1_m) && !isnan(imu_alt))
          s_base_imu1_m = imu_alt;
      }

      float agl_bmp1 = NAN, agl_imu1 = NAN, agl_fused = NAN;
      if (s_agl_ready)
      {
        if (!isnan(s_base_bmp1_m) && !isnan(bmp_alt))
          agl_bmp1 = bmp_alt - s_base_bmp1_m;
        if (!isnan(s_base_imu1_m) && !isnan(imu_alt))
          agl_imu1 = imu_alt - s_base_imu1_m;
        // Weighted fusion when both available; fallback otherwise
        if (!isnan(agl_bmp1) && !isnan(agl_imu1))
        {
          agl_fused = FUSION_W_BMP1 * agl_bmp1 + FUSION_W_IMU1 * agl_imu1;
        }
        else if (!isnan(agl_bmp1))
        {
          agl_fused = agl_bmp1;
        }
        else if (!isnan(agl_imu1))
        {
          agl_fused = agl_imu1;
        }
      }

      //* Vertical speed from AGL derivative (EMA)
      float vz = NAN;
      float dt_s_for_step = NAN;
      if (s_agl_ready && !isnan(agl_fused))
      {
        if (have_prev_alt)
        {
          float dt_ms = (float)(now - prev_ms);
          if (dt_ms < 1)
            dt_ms = 1;
          if (dt_ms > FUSION_VZ_MAX_DT_MS)
            dt_ms = FUSION_VZ_MAX_DT_MS;
          dt_s_for_step = dt_ms / 1000.0f;
          float dv = agl_fused - prev_alt;
          float inst_vz = dv / dt_s_for_step;
          if (isnan(vz_filt))
            vz_filt = inst_vz;
          vz_filt = FUSION_VZ_ALPHA * vz_filt + (1.0f - FUSION_VZ_ALPHA) * inst_vz;
          vz = vz_filt;
        }
        else
        {
          // first sample sequence
          dt_s_for_step = NAN;
        }
        prev_alt = agl_fused;
        prev_ms = now;
        have_prev_alt = true;
      }
      else
      {
        have_prev_alt = false;
        vz_filt = NAN;
      }

      //* Vertical accel from IMU1 (earth frame), and optionally integrate for vz
      float az_e_mps2 = NAN;
      if (vi)
      {
        float v_b[3] = {u1.accel_g[0] * G0, u1.accel_g[1] * G0, u1.accel_g[2] * G0};
        float q[4] = {u1.quat[0], u1.quat[1], u1.quat[2], u1.quat[3]};
        float v_e[3];
        rotate_vec_by_quat(q, v_b, v_e);
        az_e_mps2 = v_e[2] - G0; // Z-up convention
#if FUSION_USE_ACC_INT
        if (!isnan(az_e_mps2) && have_prev_alt)
        {
          float dt = isnan(dt_s_for_step) ? (FUSION_VZ_MAX_DT_MS / 1000.0f) : dt_s_for_step;
          // Leaky integration to bound drift
          const float leak = 0.02f; // per update; tune as needed
          vz_acc = (1.0f - leak) * vz_acc + az_e_mps2 * dt;
        }
        else if (!have_prev_alt)
        {
          vz_acc = 0.0f;
        }
#endif
      }

      //* Atmospherics: speed of sound from temperature (dynamic, for visibility)
      float temp_c = vb ? (float)b.temperature_c : NAN;
      float press_hPa = vb ? (float)(b.pressure_pa / 100.0) : NAN;
      float sos = NAN, mach_vz = NAN;
      if (!isnan(temp_c))
      {
        float T = temp_c + 273.15f;
        const float gamma = 1.4f;
        const float R = 287.05f;
        sos = sqrtf(gamma * R * T);
        if (!isnan(vz))
          mach_vz = fabsf(vz) / sos;
      }

      //* Conservative SoS references: compute once from ground temp and estimate at +10kft
      if (!have_sos_refs && vb)
      {
        float T0 = (float)b.temperature_c + 273.15f;
        const float gamma = 1.4f;
        const float R = 287.05f;
        sos_ground_mps = sqrtf(gamma * R * T0);
        float T10k = T0 - SOS_10KFT_DELTA_K;
        if (T10k < 150.0f)
          T10k = 150.0f; // clamp
        sos_10kft_mps = sqrtf(gamma * R * T10k);
        sos_min_mps = fmaxf(SOS_MIN_FLOOR_MPS, fminf(sos_ground_mps, sos_10kft_mps));
        have_sos_refs = true;
      }

      //* Conservative Mach proxy using worst-case tilt: computed after vz_fused
      float mach_cons = NAN;

      //* Predictive: time to apogee and predicted apogee altitude (biased early/low)
      float t_apx = NAN, z_apx = NAN;
      if (s_agl_ready && !isnan(agl_fused) && !isnan(vz))
      {
        if (vz > 0.0f)
        {
          t_apx = FUSION_SAFE_TAPX_FACTOR * (vz / G0);
          z_apx = agl_fused + FUSION_SAFE_ZAPX_FACTOR * (vz * vz) / (2.0f * G0);
        }
        else
        {
          t_apx = 0.0f;
          z_apx = agl_fused; // already descending
        }
      }

      //* Euler from IMU1
      float yaw = NAN, pitch = NAN, roll = NAN;
      float tilt_deg = NAN, tilt_az_deg = NAN, tilt_az_deg360 = NAN, tilt_az_unwrapped_deg = NAN;
      if (vi)
      {
        quat_to_euler(u1.quat[0], u1.quat[1], u1.quat[2], u1.quat[3], yaw, pitch, roll);
        // Tilt metrics robust near vertical: rotate body +X (nose) into Earth frame
        const float x_body[3] = {1.0f, 0.0f, 0.0f};
        float x_earth[3];
        float q[4] = {u1.quat[0], u1.quat[1], u1.quat[2], u1.quat[3]};
        rotate_vec_by_quat(q, x_body, x_earth);
        // Angle from Earth Up (Z)
        float cz = fmaxf(-1.0f, fminf(1.0f, x_earth[2]));
        tilt_deg = acosf(cz) * 57.2957795f;
        // Azimuth of tilt direction around Earth Z (atan2(y, x)) with smoothing and hysteresis
        float h2 = x_earth[0] * x_earth[0] + x_earth[1] * x_earth[1];
        float h = sqrtf(h2);
        if (tilt_deg >= FUSION_TILT_AZ_MIN_TILT_DEG && h > 1e-4f)
        {
          float hx = x_earth[0] / h;
          float hy = x_earth[1] / h;
          if (!have_tilt_az || isnan(tiltAzX) || isnan(tiltAzY))
          {
            tiltAzX = hx;
            tiltAzY = hy;
            have_tilt_az = true;
          }
          else
          {
            tiltAzX = FUSION_TILT_AZ_ALPHA * tiltAzX + (1.0f - FUSION_TILT_AZ_ALPHA) * hx;
            tiltAzY = FUSION_TILT_AZ_ALPHA * tiltAzY + (1.0f - FUSION_TILT_AZ_ALPHA) * hy;
            float n = sqrtf(tiltAzX * tiltAzX + tiltAzY * tiltAzY);
            if (n > 1e-6f)
            {
              tiltAzX /= n;
              tiltAzY /= n;
            }
          }
          tilt_az_deg = atan2f(tiltAzY, tiltAzX) * 57.2957795f; // East=0°, North=+90°
        }
        else
        {
          // keep last if we have one; else NaN
          if (have_tilt_az)
            tilt_az_deg = atan2f(tiltAzY, tiltAzX) * 57.2957795f;
          else
            tilt_az_deg = NAN;
        }
        // 0..360 mapping
        if (!isnan(tilt_az_deg))
        {
          tilt_az_deg360 = (tilt_az_deg < 0.0f) ? (tilt_az_deg + 360.0f) : tilt_az_deg;
          // Unwrap across ±180 into continuous angle
          if (!have_tilt_az_acc)
          {
            tilt_az_prev_deg = tilt_az_deg;
            tilt_az_unwrapped = tilt_az_deg;
            have_tilt_az_acc = true;
          }
          else
          {
            float delta = tilt_az_deg - tilt_az_prev_deg;
            // Wrap delta into [-180, 180]
            while (delta > 180.0f)
              delta -= 360.0f;
            while (delta < -180.0f)
              delta += 360.0f;
            tilt_az_unwrapped += delta;
            tilt_az_prev_deg = tilt_az_deg;
          }
          tilt_az_unwrapped_deg = tilt_az_unwrapped;
        }
      }

      // Fused vertical speed (complementary)
      float vz_fused = NAN;
      if (!isnan(vz) && !isnan(vz_acc))
      {
        vz_fused = FUSION_VZ_FUSE_BETA * vz + (1.0f - FUSION_VZ_FUSE_BETA) * vz_acc;
      }
      else if (!isnan(vz))
      {
        vz_fused = vz;
      }
      else if (!isnan(vz_acc))
      {
        vz_fused = vz_acc;
      }

      // Conservative Mach proxy now that vz_fused is known
      if (!isnan(vz_fused) && have_sos_refs)
      {
        float c = cosf(TILT_MAX_DEPLOY_DEG * 0.01745329252f);
        if (c < 0.1f)
          c = 0.1f; // avoid blow-up
        float v_body = fabsf(vz_fused) / c;
        mach_cons = v_body / sos_min_mps;
      }

      if (!s_alt_mutex)
        s_alt_mutex = xSemaphoreCreateMutex();
      if (s_alt_mutex)
        xSemaphoreTake(s_alt_mutex, portMAX_DELAY);
      s_fused_alt.stamp_ms = now;
      s_fused_alt.age_ms = 0;
      s_fused_alt.bmp1_alt_m = bmp_alt;
      s_fused_alt.imu1_alt_m = imu_alt;
      s_fused_alt.agl_bmp1_m = agl_bmp1;
      s_fused_alt.agl_imu1_m = agl_imu1;
      s_fused_alt.agl_fused_m = agl_fused;
      s_fused_alt.agl_ready = s_agl_ready;
      s_fused_alt.vz_mps = vz;
      s_fused_alt.vz_acc_mps = vz_acc;
      s_fused_alt.vz_fused_mps = vz_fused;
      s_fused_alt.az_imu1_mps2 = az_e_mps2;
      s_fused_alt.temp_c = temp_c;
      s_fused_alt.press_hPa = press_hPa;
      s_fused_alt.sos_mps = sos;
      s_fused_alt.mach_vz = mach_vz;
      s_fused_alt.sos_ground_mps = sos_ground_mps;
      s_fused_alt.sos_10kft_mps = sos_10kft_mps;
      s_fused_alt.sos_min_mps = sos_min_mps;
      s_fused_alt.mach_cons = mach_cons;
      s_fused_alt.yaw_deg = yaw;
      s_fused_alt.pitch_deg = pitch;
      s_fused_alt.roll_deg = roll;
      s_fused_alt.tilt_deg = tilt_deg;
      s_fused_alt.tilt_az_deg = tilt_az_deg;
      s_fused_alt.tilt_az_deg360 = tilt_az_deg360;
      s_fused_alt.tilt_az_unwrapped_deg = tilt_az_unwrapped_deg;
      s_fused_alt.t_apogee_s = t_apx;
      s_fused_alt.apogee_agl_m = z_apx;
      if (s_alt_mutex)
        xSemaphoreGive(s_alt_mutex);

      vTaskDelayUntil(&last, pdMS_TO_TICKS(TELEM_PERIOD_MS));
    }
  }

  void fusionStartTask()
  {
    if (!s_alt_mutex)
      s_alt_mutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(fusion_task, "fusion", 3072, nullptr, 1, nullptr, APP_CPU_NUM);
  }

  bool fusionGetAlt(FusedAlt &out)
  {
    if (s_alt_mutex)
      xSemaphoreTake(s_alt_mutex, portMAX_DELAY);
    out = s_fused_alt;
    if (s_alt_mutex)
      xSemaphoreGive(s_alt_mutex);
    return true;
  }

  // Legacy placeholders
  void fusion_init() {}
  void fusion_update() {}
  bool fusion_get(FusedImu &out)
  {
    out = s_fused_imu;
    return true;
  }

  void fusionSoftReset()
  {
    s_reset_req = true;
  }

} // namespace svc
