// Monitoring configuration (serial output modes, channel presets)
#pragma once

// Prefer telemetry snapshot when available
#ifndef MON_LOG_FROM_TELEM
#define MON_LOG_FROM_TELEM 1
#endif

// Serial plotter vs CSV (legacy)
#ifndef SERIAL_PLOTTER_MODE
#define SERIAL_PLOTTER_MODE 1   // 1 = VSCode Serial Plotter format, 0 = CSV
#endif

// Teleplot output
#ifndef TELEPLOT_MODE
#define TELEPLOT_MODE 1         // 1 = Teleplot format enabled by default
#endif
#ifndef TELEPLOT_INCLUDE_TS
#define TELEPLOT_INCLUDE_TS 0   // 1 = include millis() timestamp in Teleplot lines
#endif

// High-level serial output toggles
//  - SERIAL_DATA_ENABLE: high-rate data lines (plotting/current state)
//  - SERIAL_DEBUG_ENABLE: human-readable status/debug messages
#ifndef SERIAL_DATA_ENABLE
#define SERIAL_DATA_ENABLE 1    // default OFF to reduce serial spam
#endif
#ifndef SERIAL_DEBUG_ENABLE
#define SERIAL_DEBUG_ENABLE 1   // default ON for setup/status messages
#endif

// Map legacy DEBUG_ENABLED to SERIAL_DEBUG_ENABLE so DEBUG* macros follow
#undef DEBUG_ENABLED
#define DEBUG_ENABLED (SERIAL_DEBUG_ENABLE)

// Teleplot debug/log formatting for state/flags block
#ifndef TELEPLOT_DEBUG_BLOCK
#define TELEPLOT_DEBUG_BLOCK 1
#endif

// Single-value plot helper (used for A/B testing plot plugins)
#ifndef VIS_TILT_ONLY_MODE
#define VIS_TILT_ONLY_MODE 0     // 1 = emit only "tilt_deg:<angle>" lines
#endif
#ifndef PLOT_SINGLE_ONLY
#define PLOT_SINGLE_ONLY 0
#endif
#ifndef PLOT_SINGLE_LABEL
#define PLOT_SINGLE_LABEL "agl_fused_m"
#endif
#ifndef PLOT_SINGLE_EXPR
#define PLOT_SINGLE_EXPR (agl_ready ? f.agl_fused_m : NAN)
#endif

// Comparison helpers (kept for future use)
#ifndef PLOT_COMPARE_ACCEL
#define PLOT_COMPARE_ACCEL 1
#endif
#ifndef PLOT_ACCEL_AXES_MASK
#define PLOT_ACCEL_AXES_MASK 0x1
#endif
#ifndef PLOT_INCLUDE_DIFF
#define PLOT_INCLUDE_DIFF 1
#endif
