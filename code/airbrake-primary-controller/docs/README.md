# Airbrake Primary Controller — Documentation

This docs tree helps new contributors ramp quickly and gives us a clear, layered mental model from hardware → drivers → sensors → services (fusion/health) → telemetry snapshot → monitoring/UI.

Sections
- docs/setup.md — Dev quickstart (PlatformIO, monitor)
- docs/wiring.md — Pins, I2C/SPI wiring, addresses
- docs/data-model.md — Sensor/telemetry data types and conventions
- docs/telemetry.md — Snapshot design, versioning, present flags
- docs/monitoring.md — Serial CSV vs VS Code Serial Plotter, presets
- docs/tasks.md — Tasks, update rates, and responsibilities
- docs/mutex-map.md — Shared resources and locking rules
- docs/sensors/ — Per-device notes (USFSMAX, MPU6050, BMP390)
- docs/datasheets/ — Place PDFs/links here
- docs/contrib-style.md — Naming, layering, file layout conventions

Architecture (overview)
- HAL: pins, I2C/SPI setup, logging mutexes
- Drivers: thin wrappers for vendor libraries (no RTOS)
- Sensors: normalize device outputs to reading structs (no RTOS)
- Services: fusion, health/FDI, calibration utilities (no RTOS)
- Telemetry: builds a periodic, consistent snapshot for consumers
- Monitoring: human-readable/dev outputs backed by the snapshot
- Tasks: FreeRTOS tasks that schedule work and guard snapshots

Folder Map (current repo)
- include/: headers with types, configs, and public APIs
- src/: implementations (Arduino/FreeRTOS tasks and modules)
- lib/: vendor libraries (USFSMAX, etc.)
- docs/: this documentation tree

Where to start
- Read docs/setup.md to build/flash/monitor.
- Skim docs/data-model.md and docs/telemetry.md to understand what values exist and how they flow.
- Use docs/monitoring.md to visualize signals and compare sensors quickly.

