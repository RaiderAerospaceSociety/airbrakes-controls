# Tasks, Rates, And Responsibilities

Current tasks (names and defaults)
- Telemetry aggregator — builds the snapshot
  - File: `src/telemetry.cpp`
  - Period: `TELEM_PERIOD_MS` (default 20 ms)
- Logger — serial monitoring output (CSV or Serial Plotter format)
  - File: `src/task_logger.cpp`
  - Period: `LOGGER_PERIOD_MS` (default 50 ms)
- IMU1 (USFSMAX) reader — polls USFSMAX and snapshots local reading
  - File: `src/sensor_imu1.cpp`
  - Period: `USFS_PERIOD_MS` (default 20 ms)
- IMU2 (MPU6050) reader — polls MPU6050 and snapshots local reading
  - File: `src/sensor_imu2.cpp`
  - Period: `IMU2_PERIOD_MS` (default 20 ms)
- BMP390 reader — barometer/altitude
  - File: `src/sensors_bmp390.*`
  - Period: `BMP390_PERIOD_MS` (default 100 ms)
- LED animation
  - File: `src/task_led.cpp`
  - Period: `LED_PERIOD_MS` (default 15 ms)

Coordination model
- Sensors run at their own rates and protect a local `*_reading_t` with a mutex.
- Telemetry periodically copies the latest readings into a single snapshot (also mutex-protected).
- Logger prints from the snapshot (recommended) or directly from sensors for ad-hoc debugging.

Tuning tips
- Set `TELEM_PERIOD_MS` at or faster than the fastest quantity your algorithm monitors.
- Run logger slower than telemetry for readability and to reduce USB bandwidth.
- Keep bus I/O short and under the bus mutex; do math and logging outside of lock.
