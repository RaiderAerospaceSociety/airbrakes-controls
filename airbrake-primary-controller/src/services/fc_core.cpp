// Platform-neutral FC core implementation
#include <math.h>
#include "services/fc_core.h"
#include "services/fc.h"
#include "config/fc_config.h"

namespace svc
{
  static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

  void fc_init(FcCoreCtx &c)
  {
    c.state = FC_PREFLIGHT;
    c.flags = 0;
    c.t_state_ms = 0;
    c.t_launch_ms = 0;
    c.t_burnout_ms = 0;
    c.t_deploy_ms = 0;
    c.tilt_latched = 0;
    c.mach_ok_acc_ms = 0;
    c.tilt_bad_acc_ms = 0;
    c.liftoff_acc_ms = 0;
    c.burnout_acc_ms = 0;
    c.imu1_ok = c.bmp1_ok = c.imu2_ok = 0;
    c.imu1_good_acc = c.imu1_bad_acc = 0;
    c.bmp1_good_acc = c.bmp1_bad_acc = 0;
    c.imu2_good_acc = c.imu2_bad_acc = 0;
  }

  // forward declarations
  static void update_flags(FcCoreCtx &c, const FcInputs &in, float &out_mach);
  static void update_fsm(FcCoreCtx &c, const FcInputs &in);

  void fc_step(FcCoreCtx &c, const FcInputs &in, FcOutputs &out)
  {
    float mach = NAN;
    update_flags(c, in, mach);
    update_fsm(c, in);

    float cmd_deg = 0.0f;
    switch (c.state)
    {
    case FC_DEPLOYED: cmd_deg = FC_DEPLOY_CMD_DEG; break;
    default: cmd_deg = 0.0f; break;
    }

    out.state = c.state;
    out.flags = c.flags;
    out.airbrake_cmd_deg = cmd_deg;
    out.t_to_apogee_s = in.t_apogee_s;
    out.t_since_launch_s = (c.t_launch_ms > 0) ? ((in.now_ms - c.t_launch_ms) * 0.001f) : 0.0f;
    out.mach_cons = mach;
    out.tilt_deg = in.tilt_deg;
  }
}

// -- Internal helpers -------------------------------------------------------
namespace svc {

  static void update_flags(FcCoreCtx &c, const FcInputs &in, float &out_mach)
  {
    // Sensor validity debounce
    auto upd = [](bool sample_ok, uint8_t &ok, uint32_t &good_acc, uint32_t &bad_acc, uint32_t dt_ms_arg)
    {
      if (sample_ok) { good_acc += dt_ms_arg; bad_acc = 0; if (!ok && good_acc >= FC_SENSOR_RECOVERY_MS) ok = 1; }
      else { bad_acc += dt_ms_arg; good_acc = 0; if (ok && bad_acc >= FC_SENSOR_INVALID_MS) ok = 0; }
    };
    upd(in.imu1_valid, c.imu1_ok, c.imu1_good_acc, c.imu1_bad_acc, in.dt_ms);
    upd(in.bmp1_valid, c.bmp1_ok, c.bmp1_good_acc, c.bmp1_bad_acc, in.dt_ms);
    upd(in.imu2_valid, c.imu2_ok, c.imu2_good_acc, c.imu2_bad_acc, in.dt_ms);

    // Tilt latch and gate
    float tilt = in.tilt_deg;
    if (!isnan(tilt))
    {
      if (tilt >= FC_TILT_ABORT_DEG) {
        c.tilt_bad_acc_ms += in.dt_ms;
        if (c.tilt_bad_acc_ms >= FC_TILT_ABORT_DWELL_MS) c.tilt_latched = 1;
      } else {
        c.tilt_bad_acc_ms = 0;
      }
    }

    // Conservative Mach proxy using fixed SoS and worst-case tilt
    float vz = !isnan(in.vz_fused_mps) ? in.vz_fused_mps : in.vz_mps;
    float mach = NAN;
    if (!isnan(vz))
    {
      float cth = cosf(FC_TILT_ABORT_DEG * 0.01745329252f);
      if (cth < 0.1f) cth = 0.1f;
      float v_body = fabsf(vz) / cth;
      mach = v_body / FC_SOS_FIXED_MPS;
      // Hysteresis + dwell
      static bool mach_ok_state = false;
      float on_th = FC_MACH_MAX_FOR_DEPLOY;
      float off_th = FC_MACH_MAX_FOR_DEPLOY + FC_MACH_HYST;
      if (mach < on_th) {
        c.mach_ok_acc_ms += in.dt_ms;
        if (!mach_ok_state && c.mach_ok_acc_ms >= FC_MACH_DWELL_MS) mach_ok_state = true;
      } else if (mach > off_th) {
        c.mach_ok_acc_ms = 0;
        mach_ok_state = false;
      }
      if (mach_ok_state) c.flags |= FCF_MACH_OK; else c.flags &= ~FCF_MACH_OK;
    }
    out_mach = mach;

    // Basic baro agreement gate
    if (in.bmp1_valid && in.imu1_valid && !isnan(in.bmp1_altitude_m) && !isnan(in.imu1_altitude_m))
    {
      float diff = fabsf(in.bmp1_altitude_m - in.imu1_altitude_m);
      static uint32_t agree_acc = 0;
      if (diff <= FC_BARO_AGREE_M) {
        agree_acc += in.dt_ms;
        if (agree_acc >= FC_BARO_AGREE_MS) c.flags |= FCF_BARO_AGREE;
      } else {
        agree_acc = 0;
        c.flags &= ~FCF_BARO_AGREE;
      }
    }

    // Instantaneous flags
    if (c.imu1_ok) c.flags |= FCF_SENS_IMU1_OK; else c.flags &= ~FCF_SENS_IMU1_OK;
    if (c.bmp1_ok) c.flags |= FCF_SENS_BMP1_OK; else c.flags &= ~FCF_SENS_BMP1_OK;
    if (c.imu2_ok) c.flags |= FCF_SENS_IMU2_OK; else c.flags &= ~FCF_SENS_IMU2_OK;
    if (!c.tilt_latched && !isnan(tilt) && tilt <= FC_TILT_ABORT_DEG) c.flags |= FCF_TILT_OK; else c.flags &= ~FCF_TILT_OK;
    if (c.tilt_latched) c.flags |= FCF_TILT_LATCH; else c.flags &= ~FCF_TILT_LATCH;
  }

  static void update_fsm(FcCoreCtx &c, const FcInputs &in)
  {
    // Liftoff detection
    bool liftoff_cond = false;
    if (!isnan(in.vz_fused_mps) && in.vz_fused_mps > FC_VZ_LIFTOFF_MPS) liftoff_cond = true;
    if (!isnan(in.az_imu1_mps2) && in.az_imu1_mps2 > FC_AZ_LIFTOFF_MPS2) liftoff_cond = true;
    if (!isnan(in.agl_fused_m) && in.agl_fused_m >= FC_LIFTOFF_MIN_AGL_M) liftoff_cond = true;
    static bool liftoff_latched = false;
    if (!liftoff_latched)
    {
      if (liftoff_cond) {
        c.liftoff_acc_ms += in.dt_ms;
        if (c.liftoff_acc_ms >= FC_LIFTOFF_DWELL_MS) {
          liftoff_latched = true;
          c.t_launch_ms = in.now_ms;
          c.flags |= FCF_LIFTOFF_DET;
        }
      } else {
        c.liftoff_acc_ms = 0;
      }
    }

    // Burnout detection
    static bool burnout_latched = false;
    if (liftoff_latched && !burnout_latched)
    {
      if (!isnan(in.az_imu1_mps2) && in.az_imu1_mps2 <= FC_BURNOUT_AZ_DONE_MPS2) {
        c.burnout_acc_ms += in.dt_ms;
        if (c.burnout_acc_ms >= FC_BURNOUT_DWELL_MS) {
          burnout_latched = true;
          c.t_burnout_ms = in.now_ms;
          c.flags |= FCF_BURNOUT_DET;
        }
      } else {
        c.burnout_acc_ms = 0;
      }
    }

    // FSM transitions
    switch (c.state)
    {
    case FC_PREFLIGHT:
      if (c.tilt_latched) { c.state = FC_ABORT_LOCKOUT; c.t_state_ms = in.now_ms; break; }
      if (liftoff_latched) { c.state = FC_BOOST; c.t_state_ms = in.now_ms; }
      break;
    case FC_BOOST:
      if (c.tilt_latched) { c.state = FC_ABORT_LOCKOUT; c.t_state_ms = in.now_ms; break; }
      if (burnout_latched) { c.state = FC_POST_BURN_HOLD; c.t_state_ms = in.now_ms; }
      break;
    case FC_POST_BURN_HOLD:
      if (c.tilt_latched) { c.state = FC_ABORT_LOCKOUT; c.t_state_ms = in.now_ms; break; }
      if (in.now_ms - c.t_state_ms >= FC_BURNOUT_HOLD_MS) { c.state = FC_WINDOW; c.t_state_ms = in.now_ms; }
      break;
    case FC_WINDOW:
    {
      if (c.tilt_latched) { c.state = FC_ABORT_LOCKOUT; c.t_state_ms = in.now_ms; break; }
      bool gates = (c.flags & FCF_SENS_IMU1_OK) && (c.flags & FCF_SENS_BMP1_OK) && (c.flags & FCF_TILT_OK) && (c.flags & FCF_MACH_OK);
      if (!isnan(in.agl_fused_m) && in.agl_fused_m >= FC_MIN_DEPLOY_AGL_M)
      {
        if (!isnan(in.apogee_agl_m) && (in.apogee_agl_m >= (FC_TARGET_APOGEE_AGL_M + FC_APOGEE_HIGH_MARGIN_M)))
        {
          if (gates) { c.state = FC_DEPLOYED; c.t_deploy_ms = in.now_ms; c.t_state_ms = in.now_ms; }
        }
      }
      break;
    }
    case FC_DEPLOYED:
      if (c.tilt_latched) { c.state = FC_ABORT_LOCKOUT; c.t_state_ms = in.now_ms; break; }
      if (!isnan(in.t_apogee_s) && in.t_apogee_s <= FC_RETRACT_BEFORE_APOGEE_S) { c.state = FC_RETRACTING; c.t_state_ms = in.now_ms; }
      else if (c.t_launch_ms > 0)
      {
        float t_since_launch = (in.now_ms - c.t_launch_ms) * 0.001f;
        if (t_since_launch > (FC_EXPECTED_TTA_S * FC_EXPECTED_TTA_SCALE_TIMEOUT)) { c.state = FC_RETRACTING; c.t_state_ms = in.now_ms; }
      }
      break;
    case FC_RETRACTING:
      c.state = FC_LOCKED; c.t_state_ms = in.now_ms; break;
    case FC_LOCKED:
      break;
    case FC_ABORT_LOCKOUT:
      break;
    default:
      c.state = FC_SAFE; c.t_state_ms = in.now_ms; break;
    }
  }
}
