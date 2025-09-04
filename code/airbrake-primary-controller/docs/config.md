Build Config And Tunables

Where
- Global app config: `include/app_config.h` (task periods, smoothing, safety constants).
- Sensor‑specific config: `include/config/*.h` (e.g., IMU2 orientation matrix).
- PlatformIO build flags: `platformio.ini` → `build_flags` to override macros at compile time.

Common Flags (units shown)
- Periods (ms): `TELEM_PERIOD_MS`, `LOGGER_PERIOD_MS`, `USFS_PERIOD_MS`, `IMU2_PERIOD_MS`, `BMP390_PERIOD_MS`.
- AGL baseline: `ZERO_AGL_AFTER_MS` (ms from boot).
- Vertical speed smoothing: `FUSION_VZ_ALPHA` (0..1), `FUSION_VZ_MAX_DT_MS`.
- Accel integration (experimental): `FUSION_USE_ACC_INT` (0/1), fusion weight `FUSION_VZ_FUSE_BETA` (0..1).
- Tilt azimuth smoothing: `FUSION_TILT_AZ_ALPHA` (0..1), `FUSION_TILT_AZ_MIN_TILT_DEG` (deg).
- Conservative Mach proxy: `TILT_MAX_DEPLOY_DEG` (deg), `SOS_10KFT_DELTA_K` (K), `SOS_MIN_FLOOR_MPS` (m/s).

IMU Orientation (body frame)
- IMU2: set a fixed 3×3 body rotation matrix in `include/config/sensors_config.h` (`IMU2_Rij`). Defaults to identity.
- IMU1: library provides a quaternion; mount the board nose‑aligned if possible.

Overriding Values
- Example: set AGL baseline to 8 s in `platformio.ini` build flags:
  - `build_flags = -D ZERO_AGL_AFTER_MS=8000`

Guidelines
- Document any added flag with units and rationale.
- Avoid changing defaults without updating `docs/monitoring.md` and the root README.
