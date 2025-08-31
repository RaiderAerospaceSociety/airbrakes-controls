# Contribution And Style Guide

Layering
- HAL (pins, buses, logging) → Drivers → Sensors → Services (fusion/health/calibration) → Telemetry → Monitoring → Tasks (scheduling only)
- Lower layers must not depend on higher layers.

Naming
- Files: `task_*.cpp` (RTOS tasks), `drv_*.cpp` (device drivers), `sensors/*.cpp`, `services/*.cpp`, `telemetry/*.cpp`, `hal/*.cpp`.
- Types: `FooReading`, `FooConfig`, `FooState`.
- Functions: `xxx_start_task()`, `xxx_get(...)`, `drv_xxx_*`, `svc_xxx_*`.

Config
- Defaults in headers under `include/config/` (to be introduced).
- Experiment overrides via PlatformIO `build_flags` (`-D NAME=VALUE`).

Docs
- Keep docs updated when adding fields or changing task rates.
- Note TelemetryRecord changes in docs/telemetry.md and bump the version in `include/telemetry.h` when the layout changes.

Concurrency
- Keep I/O under bus mutexes short; do computation outside locks.
- Snapshots (sensor or telemetry) are copied under a local mutex.
- Avoid nested locks; document any lock ordering if necessary.

