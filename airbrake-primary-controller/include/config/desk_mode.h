// ===== Desk Mode Overrides =====
// Brief: Scales down thresholds/delays so you can test on a desk.
// Usage: build with -D DESK_MODE=1 (e.g., in platformio.ini build_flags)
#pragma once
#if defined(DESK_MODE) && DESK_MODE

// --- App-level tweaks ---
#undef ZERO_AGL_AFTER_MS
#define ZERO_AGL_AFTER_MS 1500

#undef FUSION_VZ_MAX_DT_MS
#define FUSION_VZ_MAX_DT_MS 100

// Optional: include timestamp in monitor lines
#undef MON_INCLUDE_TS
#define MON_INCLUDE_TS 1

// --- Flight controller thresholds (smaller/easier on bench) ---
// Tilt: make abort unlikely on the bench
#undef FC_TILT_ABORT_DEG
#define FC_TILT_ABORT_DEG 75.0f

// Liftoff: allow tiny motion to trigger
#undef FC_VZ_LIFTOFF_MPS
#define FC_VZ_LIFTOFF_MPS 0.5f
#undef FC_AZ_LIFTOFF_MPS2
#define FC_AZ_LIFTOFF_MPS2 1.0f
#undef FC_LIFTOFF_MIN_AGL_M
#define FC_LIFTOFF_MIN_AGL_M 0.20f
#undef FC_LIFTOFF_DWELL_MS
#define FC_LIFTOFF_DWELL_MS 50

// Burnout: quicker and lower threshold
#undef FC_BURNOUT_AZ_DONE_MPS2
#define FC_BURNOUT_AZ_DONE_MPS2 0.3f
#undef FC_BURNOUT_DWELL_MS
#define FC_BURNOUT_DWELL_MS 120
#undef FC_BURNOUT_HOLD_MS
#define FC_BURNOUT_HOLD_MS 400

// Deployment window guards: very low altitude/target for bench
#undef FC_MIN_DEPLOY_AGL_M
#define FC_MIN_DEPLOY_AGL_M 0.20f // ~20 cm
#undef FC_TARGET_APOGEE_AGL_M
#define FC_TARGET_APOGEE_AGL_M 0.25f // ~25 cm
#undef FC_APOGEE_HIGH_MARGIN_M
#define FC_APOGEE_HIGH_MARGIN_M 0.05f

// Retraction timing: much faster
#undef FC_RETRACT_BEFORE_APOGEE_S
#define FC_RETRACT_BEFORE_APOGEE_S 0.5f
#undef FC_EXPECTED_TTA_S
#define FC_EXPECTED_TTA_S 3.0f
#undef FC_EXPECTED_TTA_SCALE_TIMEOUT
#define FC_EXPECTED_TTA_SCALE_TIMEOUT 1.1f

// Sensor validity: reduce dwell to react quickly
#undef FC_SENSOR_INVALID_MS
#define FC_SENSOR_INVALID_MS 80
#undef FC_SENSOR_RECOVERY_MS
#define FC_SENSOR_RECOVERY_MS 200

// Mach dwell: make the gate settle quickly
#undef FC_MACH_DWELL_MS
#define FC_MACH_DWELL_MS 50

// Servo command: smaller movement for safety on the bench
#undef FC_DEPLOY_CMD_DEG
#define FC_DEPLOY_CMD_DEG 10.0f

#endif // DESK_MODE
