# Contribution And Style Guide

Layering
- HAL (pins, buses, logging) → Drivers → Sensors → Services (fusion/health/calibration) → Telemetry → Monitoring → Tasks (scheduling only)
- Lower layers must not depend on higher layers.

Naming and Case

Comment Conventions
- File banner (top of each .h/.cpp):
  - `// ===== <Module Name> =====`
  - `// Brief: one-line summary`
  - `// Refs: docs/...`
- Section markers (vanilla, readable without plugins):
  - `//* -- Includes --`
  - `//* -- Globals --`
  - `//* -- Task --`
  - `//* -- API --`
  - Keep markers short and consistent.
- Better Comments coloring (optional but helpful):
  - `//!` Alert/Caveat (red)
  - `//*` Highlight/Section header (green)
  - `//?` Question (blue)
  - `// TODO(owner, yyyy-mm-dd):` actionable items (orange)
  - `////` Deprecated/disabled blocks
  - Normal notes use `// Note: ...`

- Files (singular nouns by context): `sensor_imu1.cpp`, `sensor_bmp390.cpp`, `service_fusion.cpp`, `task_logger.cpp`.
- Types (PascalCase): `Imu1Reading`, `BmpReading`, `TelemetryRecord`, `FusedAlt`.
- Functions (camelCase, verbNoun): `imu1StartTask()`, `imu1Get()`, `bmp390StartTask()`, `fusionGetAlt()`.
- Variables (camelCase, nouns; booleans prefixed with is/has/should): `lastPressurePa`, `isValid`, `hasBaseline`.
- Constants/macros (UPPER_SNAKE_CASE): `ZERO_AGL_AFTER_MS`, `FUSION_VZ_ALPHA`.
- Enums (PascalCase types, UPPER_SNAKE_CASE members): `enum TelemetryPresent { TP_IMU1 = 1u<<1, ... }`.

Units and Frames
- Always document units and frames in public headers (Doxygen). Examples:
  - `/// Altitude above MSL (meters), Earth frame`
  - `/// Acceleration (g) in body frame (+X nose, +Y right, +Z down)`
- Prefer consistent suffixes in plot labels: `_m`, `_mps`, `_Pa`, `_deg`, `_C`, `_hPa`.
- Earth frame: ENU (+Z up) for vertical calculations. Body frame: +X forward (nose), +Y right, +Z down.

Public API (project-owned only)
- Sensor tasks: `xxxStartTask()` and getters: `xxxGet(ReadingType& out)->bool`.
- Reading types: `XxxReading` containing all values from a device, with `valid` and (optional) `timestampMs`.
- Keep legacy C-style wrappers temporarily if needed for compatibility, but prefer the camelCase API going forward.

Doxygen Comments (hoverable docs)
- Functions:
  - `/** @brief Get latest IMU1 snapshot.
      @param out Filled with latest reading (see units in struct docs).
      @return true if data valid. */`
- Struct fields:
  - `/// Vertical speed (m/s), Earth frame (+Z up)`

Config and Build Flags
- Centralize tunables in `include/app_config.h` and `include/config/*.h` (by domain).
- Overrides via PlatformIO `build_flags` (e.g., `-D ZERO_AGL_AFTER_MS=8000`).
- Document each flag with units and rationale.

Concurrency
- Keep I/O under bus mutexes; do math/logging outside locks.
- Snapshots are copied under a local mutex; never expose internal storage by reference.
- Avoid nested locks; document any lock ordering if necessary.
