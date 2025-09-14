// Flight Controller core (platform-neutral)
#pragma once

#include <stdint.h>

namespace svc
{

  // Inputs consumed each tick by the FC core
  struct FcInputs
  {
    // Timing
    uint32_t dt_ms;     // delta time since previous step
    uint32_t now_ms;    // absolute time (ms)

    // Fused values
    float tilt_deg;         // deg
    float agl_fused_m;      // m
    float vz_fused_mps;     // m/s
    float vz_mps;           // m/s (fallback when fused not available)
    float az_imu1_mps2;     // m/s^2 (earth Z)
    float t_apogee_s;       // s (biased-early)
    float apogee_agl_m;     // m (biased-low)
    uint8_t agl_ready;      // 1 if fused baseline captured

    // Raw altitudes for agreement gate
    float bmp1_altitude_m;  // m
    float imu1_altitude_m;  // m (internal baro)

    // Sensor validity flags (raw device sampling ok)
    uint8_t imu1_valid;
    uint8_t bmp1_valid;
    uint8_t imu2_valid;
  };

  // Outputs produced each tick by the FC core
  struct FcOutputs
  {
    uint8_t  state;           // FcState
    uint32_t flags;           // FcFlags
    float    airbrake_cmd_deg;// deg
    float    t_since_launch_s;// s
    float    t_to_apogee_s;   // s
    float    mach_cons;       // unitless
    float    tilt_deg;        // deg (echo for convenience)
  };

  // Internal core context (persistent between steps)
  struct FcCoreCtx
  {
    // State machine
    uint8_t  state;         // current FcState
    uint32_t flags;         // bitmask FcFlags
    uint32_t t_state_ms;
    uint32_t t_launch_ms;
    uint32_t t_burnout_ms;
    uint32_t t_deploy_ms;
    uint8_t  tilt_latched;

    // Debounce accumulators
    uint32_t mach_ok_acc_ms;
    uint32_t tilt_bad_acc_ms;
    uint32_t liftoff_acc_ms;
    uint32_t burnout_acc_ms;

    // Sensor validity debounce
    uint8_t  imu1_ok, bmp1_ok, imu2_ok;
    uint32_t imu1_good_acc, imu1_bad_acc;
    uint32_t bmp1_good_acc, bmp1_bad_acc;
    uint32_t imu2_good_acc, imu2_bad_acc;
  };

  // Initialize/reset the FC core context
  void fc_init(FcCoreCtx &ctx);

  // Single tick: update flags/FSM and produce outputs
  void fc_step(FcCoreCtx &ctx, const FcInputs &in, FcOutputs &out);

} // namespace svc

