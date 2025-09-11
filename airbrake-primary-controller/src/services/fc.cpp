// Airbrake Flight Controller (FSM) implementation
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "services/fc.h"
#include "services/fusion.h"
#include "sensor_bmp390.h"
#include "sensor_imu1.h"
#include "sensor_imu2.h"
#include "config/fc_config.h"
#include "app_config.h"

namespace svc
{

  static SemaphoreHandle_t s_mutex = nullptr;
  static FcStatus s_stat = {};

  // Internal state
  static FcState s_state = FC_PREFLIGHT;
  static uint32_t s_flags = 0;
  static uint32_t t_state_ms = 0;
  static uint32_t t_launch_ms = 0;
  static uint32_t t_burnout_ms = 0;
  static uint32_t t_deploy_ms = 0;
  static bool tilt_latched = false;

  // Debounce accumulators
  static uint32_t mach_ok_acc_ms = 0;
  static uint32_t tilt_bad_acc_ms = 0;
  static uint32_t liftoff_acc_ms = 0;
  static uint32_t burnout_acc_ms = 0;

  // Sensor validity debounce
  static bool imu1_ok = false, bmp1_ok = false, imu2_ok = false;
  static uint32_t imu1_good_acc = 0, imu1_bad_acc = 0;
  static uint32_t bmp1_good_acc = 0, bmp1_bad_acc = 0;
  static uint32_t imu2_good_acc = 0, imu2_bad_acc = 0;

  // Helpers
  static inline float now_s(uint32_t now_ms) { return now_ms * 0.001f; }
  static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

  static void fc_update_flags(const svc::FusedAlt &f, uint32_t dt_ms)
  {
    // Sensor validity
    bmp_reading_t b;
    bool vb = bmp390Get(b) && b.valid;
    imu1_reading_t u1;
    bool v1 = imu1Get(u1) && u1.valid;
    imu2_reading_t u2;
    bool v2 = imu2Get(u2) && u2.valid;
    // Debounce logic: require sustained good to set OK; sustained bad to clear
    auto upd = [](bool sample_ok, bool &ok, uint32_t &good_acc, uint32_t &bad_acc, uint32_t dt_ms_arg)
    {
      if (sample_ok)
      {
        good_acc += dt_ms_arg;
        bad_acc = 0;
        if (!ok && good_acc >= FC_SENSOR_RECOVERY_MS)
          ok = true;
      }
      else
      {
        bad_acc += dt_ms_arg;
        good_acc = 0;
        if (ok && bad_acc >= FC_SENSOR_INVALID_MS)
          ok = false;
      }
    };
    upd(v1, imu1_ok, imu1_good_acc, imu1_bad_acc, dt_ms);
    upd(vb, bmp1_ok, bmp1_good_acc, bmp1_bad_acc, dt_ms);
    upd(v2, imu2_ok, imu2_good_acc, imu2_bad_acc, dt_ms);

    // Tilt latch and gate
    float tilt = f.tilt_deg;
    if (!isnan(tilt))
    {
      if (tilt >= FC_TILT_ABORT_DEG)
      {
        tilt_bad_acc_ms += dt_ms;
        if (tilt_bad_acc_ms >= FC_TILT_ABORT_DWELL_MS)
          tilt_latched = true;
      }
      else
      {
        tilt_bad_acc_ms = 0; // below threshold
      }
    }

    // Conservative Mach proxy using fixed SoS and worst-case tilt
    float vz = !isnan(f.vz_fused_mps) ? f.vz_fused_mps : f.vz_mps;
    float mach = NAN;
    if (!isnan(vz))
    {
      float c = cosf(FC_TILT_ABORT_DEG * 0.01745329252f);
      if (c < 0.1f)
        c = 0.1f;
      float v_body = fabsf(vz) / c;
      mach = v_body / FC_SOS_FIXED_MPS;
      // Hysteresis + dwell
      static bool mach_ok_state = false;
      float on_th = FC_MACH_MAX_FOR_DEPLOY;
      float off_th = FC_MACH_MAX_FOR_DEPLOY + FC_MACH_HYST;
      if (mach < on_th)
      {
        mach_ok_acc_ms += dt_ms;
        if (!mach_ok_state && mach_ok_acc_ms >= FC_MACH_DWELL_MS)
          mach_ok_state = true;
      }
      else if (mach > off_th)
      {
        mach_ok_acc_ms = 0;
        mach_ok_state = false;
      }
      if (mach_ok_state)
        s_flags |= FCF_MACH_OK;
      else
        s_flags &= ~FCF_MACH_OK;
    }

    // Basic baro agreement (optional gate)
    if (vb && v1 && !isnan(b.altitude_m) && !isnan(u1.altitude_m))
    {
      float diff = fabsf((float)b.altitude_m - u1.altitude_m);
      static uint32_t agree_acc = 0;
      if (diff <= FC_BARO_AGREE_M)
      {
        agree_acc += dt_ms;
        if (agree_acc >= FC_BARO_AGREE_MS)
          s_flags |= FCF_BARO_AGREE;
      }
      else
      {
        agree_acc = 0;
        s_flags &= ~FCF_BARO_AGREE;
      }
    }

    // Write instantaneous values into flags
    if (imu1_ok)
      s_flags |= FCF_SENS_IMU1_OK;
    else
      s_flags &= ~FCF_SENS_IMU1_OK;
    if (bmp1_ok)
      s_flags |= FCF_SENS_BMP1_OK;
    else
      s_flags &= ~FCF_SENS_BMP1_OK;
    if (imu2_ok)
      s_flags |= FCF_SENS_IMU2_OK;
    else
      s_flags &= ~FCF_SENS_IMU2_OK;
    if (!tilt_latched && !isnan(tilt) && tilt <= FC_TILT_ABORT_DEG)
      s_flags |= FCF_TILT_OK;
    else
      s_flags &= ~FCF_TILT_OK;
    if (tilt_latched)
      s_flags |= FCF_TILT_LATCH;
    else
      s_flags &= ~FCF_TILT_LATCH;

    // Update published instantaneous metrics
    if (s_mutex)
      xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_stat.mach_cons = mach;
    s_stat.tilt_deg = tilt;
    if (s_mutex)
      xSemaphoreGive(s_mutex);
  }

  static void fc_update_fsm(const svc::FusedAlt &f, uint32_t now_ms, uint32_t dt_ms)
  {
    // Liftoff detection
    bool liftoff_cond = false;
    if (!isnan(f.vz_fused_mps) && f.vz_fused_mps > FC_VZ_LIFTOFF_MPS)
      liftoff_cond = true;
    if (!isnan(f.az_imu1_mps2) && f.az_imu1_mps2 > FC_AZ_LIFTOFF_MPS2)
      liftoff_cond = true;
    if (!isnan(f.agl_fused_m) && f.agl_fused_m >= FC_LIFTOFF_MIN_AGL_M)
      liftoff_cond = true;
    static bool liftoff_latched = false;
    if (!liftoff_latched)
    {
      if (liftoff_cond)
      {
        liftoff_acc_ms += dt_ms;
        if (liftoff_acc_ms >= FC_LIFTOFF_DWELL_MS)
        {
          liftoff_latched = true;
          t_launch_ms = now_ms;
          s_flags |= FCF_LIFTOFF_DET;
        }
      }
      else
      {
        liftoff_acc_ms = 0;
      }
    }

    // Burnout detection (vertical accel near 0 or negative)
    static bool burnout_latched = false;
    if (liftoff_latched && !burnout_latched)
    {
      if (!isnan(f.az_imu1_mps2) && f.az_imu1_mps2 <= FC_BURNOUT_AZ_DONE_MPS2)
      {
        burnout_acc_ms += dt_ms;
        if (burnout_acc_ms >= FC_BURNOUT_DWELL_MS)
        {
          burnout_latched = true;
          t_burnout_ms = now_ms;
          s_flags |= FCF_BURNOUT_DET;
        }
      }
      else
      {
        burnout_acc_ms = 0;
      }
    }

    // FSM transitions
    switch (s_state)
    {
    case FC_PREFLIGHT:
      if (tilt_latched)
      {
        s_state = FC_ABORT_LOCKOUT;
        t_state_ms = now_ms;
        break;
      }
      // Stay in preflight until liftoff
      if (liftoff_latched)
      {
        s_state = FC_BOOST;
        t_state_ms = now_ms;
      }
      break;
    case FC_BOOST:
      if (tilt_latched)
      {
        s_state = FC_ABORT_LOCKOUT;
        t_state_ms = now_ms;
        break;
      }
      if (burnout_latched)
      {
        s_state = FC_POST_BURN_HOLD;
        t_state_ms = now_ms;
      }
      break;
    case FC_POST_BURN_HOLD:
      if (tilt_latched)
      {
        s_state = FC_ABORT_LOCKOUT;
        t_state_ms = now_ms;
        break;
      }
      if (now_ms - t_state_ms >= FC_BURNOUT_HOLD_MS)
      {
        s_state = FC_WINDOW;
        t_state_ms = now_ms;
      }
      break;
    case FC_WINDOW:
    {
      if (tilt_latched)
      {
        s_state = FC_ABORT_LOCKOUT;
        t_state_ms = now_ms;
        break;
      }
      bool gates = (s_flags & FCF_SENS_IMU1_OK) && (s_flags & FCF_SENS_BMP1_OK) &&
                   (s_flags & FCF_TILT_OK) && (s_flags & FCF_MACH_OK);
      if (!isnan(f.agl_fused_m) && f.agl_fused_m >= FC_MIN_DEPLOY_AGL_M)
      {
        if (!isnan(f.apogee_agl_m) && (f.apogee_agl_m >= (FC_TARGET_APOGEE_AGL_M + FC_APOGEE_HIGH_MARGIN_M)))
        {
          if (gates)
          {
            s_state = FC_DEPLOYED;
            t_deploy_ms = now_ms;
            t_state_ms = now_ms;
          }
        }
      }
      break;
    }
    case FC_DEPLOYED:
      if (tilt_latched)
      {
        s_state = FC_ABORT_LOCKOUT;
        t_state_ms = now_ms;
        break;
      }
      // Retract early based on biased-early apogee estimator or timeout
      if (!isnan(f.t_apogee_s) && f.t_apogee_s <= FC_RETRACT_BEFORE_APOGEE_S)
      {
        s_state = FC_RETRACTING;
        t_state_ms = now_ms;
      }
      else if (t_launch_ms > 0)
      {
        float t_since_launch = (now_ms - t_launch_ms) * 0.001f;
        if (t_since_launch > (FC_EXPECTED_TTA_S * FC_EXPECTED_TTA_SCALE_TIMEOUT))
        {
          s_state = FC_RETRACTING;
          t_state_ms = now_ms;
        }
      }
      break;
    case FC_RETRACTING:
      // For now, immediate move to locked after command issued
      s_state = FC_LOCKED;
      t_state_ms = now_ms;
      break;
    case FC_LOCKED:
      // Do nothing
      break;
    case FC_ABORT_LOCKOUT:
      // Stay here
      break;
    default:
      s_state = FC_SAFE;
      t_state_ms = now_ms;
      break;
    }

    // Commands
    float cmd_deg = 0.0f;
    switch (s_state)
    {
    case FC_DEPLOYED:
      cmd_deg = FC_DEPLOY_CMD_DEG;
      break;
    case FC_RETRACTING:
    case FC_LOCKED:
    case FC_ABORT_LOCKOUT:
    case FC_BOOST:
    case FC_POST_BURN_HOLD:
    case FC_WINDOW:
    case FC_SAFE:
    case FC_PREFLIGHT:
    case FC_ARMED_WAIT:
    default:
      cmd_deg = 0.0f;
      break;
    }

    // Publish status snapshot
    if (s_mutex)
      xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_stat.stamp_ms = now_ms;
    s_stat.state = static_cast<uint8_t>(s_state);
    s_stat.flags = s_flags;
    s_stat.t_to_apogee_s = f.t_apogee_s;
    s_stat.t_since_launch_s = (t_launch_ms > 0) ? ((now_ms - t_launch_ms) * 0.001f) : 0.0f;
    s_stat.airbrake_cmd_deg = cmd_deg;
    if (s_mutex)
      xSemaphoreGive(s_mutex);
  }

  static void fc_task(void *param)
  {
    const TickType_t period = pdMS_TO_TICKS(TELEM_PERIOD_MS);
    TickType_t last = xTaskGetTickCount();
    uint32_t prev_ms = millis();
    if (!s_mutex)
      s_mutex = xSemaphoreCreateMutex();
    for (;;)
    {
      FusedAlt f;
      svc::fusionGetAlt(f);
      uint32_t now = millis();
      uint32_t dt = now - prev_ms;
      if (dt > 1000)
        dt = 1000;
      if (dt < 1)
        dt = 1;
      prev_ms = now;
      // Update gates/flags then FSM
      fc_update_flags(f, dt);
      fc_update_fsm(f, now, dt);
      vTaskDelayUntil(&last, period);
    }
  }

  void fcStartTask()
  {
    if (!s_mutex)
      s_mutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(fc_task, "fc", 4096, nullptr, TASK_PRIO_LOGGER, nullptr, APP_CPU_NUM);
  }

  bool fcGetStatus(FcStatus &out)
  {
    if (s_mutex)
      xSemaphoreTake(s_mutex, portMAX_DELAY);
    out = s_stat;
    if (s_mutex)
      xSemaphoreGive(s_mutex);
    return true;
  }

} // namespace svc
