// Flight Controller (airbrakes FSM) configuration
#pragma once

// Units are documented per field. Override via build_flags as needed.

// SOS used for Mach gating (fixed, conservative)
#ifndef FC_SOS_FIXED_MPS
#define FC_SOS_FIXED_MPS 300.0f
#endif

// Mach gating
#ifndef FC_MACH_MAX_FOR_DEPLOY
#define FC_MACH_MAX_FOR_DEPLOY 0.50f
#endif
#ifndef FC_MACH_HYST
#define FC_MACH_HYST 0.02f
#endif
#ifndef FC_MACH_DWELL_MS
#define FC_MACH_DWELL_MS 300
#endif

// Tilt limits
#ifndef FC_TILT_ABORT_DEG
#define FC_TILT_ABORT_DEG 30.0f
#endif
#ifndef FC_TILT_ABORT_DWELL_MS
#define FC_TILT_ABORT_DWELL_MS 200
#endif

// Liftoff detection
#ifndef FC_VZ_LIFTOFF_MPS
#define FC_VZ_LIFTOFF_MPS 8.0f
#endif
#ifndef FC_AZ_LIFTOFF_MPS2
#define FC_AZ_LIFTOFF_MPS2 15.0f
#endif
#ifndef FC_LIFTOFF_MIN_AGL_M
#define FC_LIFTOFF_MIN_AGL_M 5.0f
#endif
#ifndef FC_LIFTOFF_DWELL_MS
#define FC_LIFTOFF_DWELL_MS 150
#endif

// Burnout detection (vertical accel near 0 or negative)
#ifndef FC_BURNOUT_AZ_DONE_MPS2
#define FC_BURNOUT_AZ_DONE_MPS2 1.0f
#endif
#ifndef FC_BURNOUT_DWELL_MS
#define FC_BURNOUT_DWELL_MS 200
#endif
#ifndef FC_BURNOUT_HOLD_MS
#define FC_BURNOUT_HOLD_MS 1500
#endif

// Deployment window guards
#ifndef FC_MIN_DEPLOY_AGL_M
#define FC_MIN_DEPLOY_AGL_M 200.0f
#endif
#ifndef FC_TARGET_APOGEE_AGL_M
#define FC_TARGET_APOGEE_AGL_M 3048.0f // 10,000 ft
#endif
#ifndef FC_APOGEE_HIGH_MARGIN_M
#define FC_APOGEE_HIGH_MARGIN_M 45.0f // ~150 ft
#endif

// Retraction timing
#ifndef FC_RETRACT_BEFORE_APOGEE_S
#define FC_RETRACT_BEFORE_APOGEE_S 5.0f
#endif
#ifndef FC_EXPECTED_TTA_S
#define FC_EXPECTED_TTA_S 18.0f
#endif
#ifndef FC_EXPECTED_TTA_SCALE_TIMEOUT
#define FC_EXPECTED_TTA_SCALE_TIMEOUT 1.2f
#endif

// Sensor validity dwell and recovery
#ifndef FC_SENSOR_INVALID_MS
#define FC_SENSOR_INVALID_MS 150
#endif
#ifndef FC_SENSOR_RECOVERY_MS
#define FC_SENSOR_RECOVERY_MS 1500
#endif

// Baro agreement gate thresholds (magnitude and dwell)
#ifndef FC_BARO_AGREE_M
#define FC_BARO_AGREE_M 15.0f
#endif
#ifndef FC_BARO_AGREE_MS
#define FC_BARO_AGREE_MS 500
#endif

// Airbrake command (placeholder; servo mapping added later)
#ifndef FC_DEPLOY_CMD_DEG
#define FC_DEPLOY_CMD_DEG 30.0f
#endif

// Optional: override values for bench testing (compile with -D DESK_MODE=1)
#if defined(DESK_MODE) && DESK_MODE
#include "config/desk_mode.h"
#endif
