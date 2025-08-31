# Mutex Map And Locking Rules

Global mutexes
- `g_i2c_mutex` (bus)
  - Owner: bus layer
  - Scope: protect Wire transactions only (beginTransmission/endTransmission, read/write)
  - Rule: hold for the shortest time; do not log while held
- `g_spi_mutex` (bus)
  - Owner: bus layer
  - Scope: protect SPI transactions only
- `g_log_mutex` (logging)
  - Owner: logging layer
  - Scope: protect Serial prints to avoid interleaving across tasks

Module-local mutexes
- IMU1 snapshot mutex (s_usfs_mutex)
  - Scope: protect read/write of the latest `imu1_reading_t`
- IMU2 snapshot mutex (s_mutex)
  - Scope: protect read/write of the latest `imu2_reading_t`
- Telemetry snapshot mutex (s_telem_mutex)
  - Scope: protect read/write of the latest `TelemetryRecord`

Lock ordering and best practices
- Avoid holding more than one mutex at a time.
- If unavoidable, define and document a strict order (e.g., bus → module → telemetry) and keep sections short.
- No logging while holding bus mutexes.
- Copy structs inside a lock, then release and process/format outside the lock.

Checklist for new code
- Does this function do I/O? Guard with the appropriate bus mutex.
- Does this function read/write a shared struct? Guard with its module mutex.
- Is any Serial print inside a lock? Consider moving it outside.

