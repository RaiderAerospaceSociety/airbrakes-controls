Architecture Overview

Flow
- Sensors (device tasks) → Fusion (derived values) → Telemetry (snapshot) → Logger/Plotter

Modules
- Sensors
  - IMU1 (USFSMAX): quaternion, accel (g), internal baro; task polls and snapshots under mutex.
  - IMU2 (MPU6050): accel (g), gyro (deg/s), temp; task polls and snapshots under mutex; fixed orientation mapping to body frame.
  - BMP390: pressure (Pa), temperature (C), altitude (m); task polls over SPI.
- Fusion Service
  - Baselines and AGL (zeroed after `ZERO_AGL_AFTER_MS`).
  - Vertical speed (baro derivative), accel‑integrated vz (optional), fused vz (complementary).
  - Atmospherics: speed of sound (dynamic) and conservative SoS/Mach for gating.
  - Attitude display: YPR (from quaternion), robust tilt angle from vertical; optional tilt azimuth (smoothed).
- Telemetry
  - Periodically builds a unified snapshot of current readings (mutex‑guarded), with present flags for subsystems.
  - Used for monitoring and, later, SD logging.
- Logger
  - Emits a single VS Code Serial Plotter line at `LOGGER_PERIOD_MS` with labeled values.

Threading and Buses
- Shared I2C/SPI guarded by global mutexes; sensors keep I/O short.
- Each module stores a local snapshot under a module mutex; getters copy out values.

Frames and Units
- Earth frame: ENU (+Z up) for vertical quantities.
- Body frame: +X forward (nose), +Y right, +Z down; used for accel rotation and tilt computation.
- All public fields note units; plot labels include unit suffixes.

Extending
- Add new sensors under `include/` and `src/` with the same task/getter pattern.
- Add derived signals in Fusion; expose via a single snapshot struct and logger channel.
