// Airbrake Flight Controller (FSM) API
#pragma once

#include <Arduino.h>

namespace svc
{

  enum FcState : uint8_t
  {
    FC_SAFE = 0,
    FC_PREFLIGHT,
    FC_ARMED_WAIT,
    FC_BOOST,
    FC_POST_BURN_HOLD,
    FC_WINDOW,
    FC_DEPLOYED,
    FC_RETRACTING,
    FC_LOCKED,
    FC_ABORT_LOCKOUT,
  };

  // Controller flags (gates and events)
  enum FcFlags : uint32_t
  {
    FCF_NONE = 0u,
    FCF_SENS_IMU1_OK = 1u << 0,
    FCF_SENS_BMP1_OK = 1u << 1,
    FCF_SENS_IMU2_OK = 1u << 2,
    FCF_BARO_AGREE = 1u << 3,
    FCF_MACH_OK = 1u << 4,
    FCF_TILT_OK = 1u << 5,
    FCF_TILT_LATCH = 1u << 6,
    FCF_LIFTOFF_DET = 1u << 7,
    FCF_BURNOUT_DET = 1u << 8,
  };

  struct FcStatus
  {
    uint32_t stamp_ms;
    uint8_t state;          // FcState
    uint32_t flags;         // FcFlags
    float mach_cons;        // conservative Mach proxy
    float tilt_deg;         // angle from vertical
    float t_since_launch_s; // seconds since liftoff (if latched), else 0
    float t_to_apogee_s;    // fusion estimate
    float airbrake_cmd_deg; // command angle (deg), 0=retracted
  };

  /** Start the FC task. */
  void fcStartTask();

  /** Copy latest FC status snapshot. */
  bool fcGetStatus(FcStatus &out);

  /** @brief Reset FC core/state to SAFE and clear flags/timers. */
  void fcSoftReset();

} // namespace svc
