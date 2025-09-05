Airbrake Primary Controller (FeatherS3)

This repo runs the airbrake controller on an ESP32‑S3 (Unexpected Maker FeatherS3). It reads multiple sensors (barometers + IMUs), derives fused altitude and vertical speed, and outputs live telemetry for plotting. Upcoming work adds airbrake actuation logic (deploy/retract) gated by safe conditions.

Quickstart
- Install VS Code + PlatformIO extension.
- Connect the FeatherS3 over USB‑C.
- Build and flash: PlatformIO toolbar → Upload (or `pio run -e um_feathers3 -t upload`).
- Open a serial monitor/plotter at 115200 baud. The VS Code Serial Plotter is supported; lines begin with `>` and contain `label:value` pairs.

What you can see live
- Altitude (BMP390, IMU1 baro) and fused AGL (zeroed after boot).
- Vertical speeds: baro‑derived, accel‑integrated, and fused.
- Attitude: yaw/pitch/roll (display only), robust tilt angle from vertical.
- Atmospherics: temperature, pressure, speed of sound, conservative Mach proxy.

Repository layout
- `include/` — Public headers, configs, and types (units documented).
- `src/` — Implementations (FreeRTOS tasks and services).
- `lib/` — Vendor libraries (e.g., USFSMAX, Adafruit drivers).
- `docs/` — Guides: setup, wiring, monitoring, data model, and conventions.

Start here
- Setup and wiring: [`docs/setup.md`](docs/setup.md), [`docs/wiring.md`](docs/wiring.md).
- Signals and plotting: [`docs/monitoring.md`](docs/monitoring.md) and [`docs/signals.md`](docs/signals.md).
- Architecture and data flow: [`docs/architecture.md`](docs/architecture.md).
- Coding conventions and style: [`docs/contrib-style.md`](docs/contrib-style.md) (naming + units).

Configuration
- Most tunables live in [`include/app_config.h`](include/app_config.h) (periods, smoothing constants, safety limits).
- Build‑time overrides can be set via PlatformIO `build_flags`.
- See [`docs/config.md`](docs/config.md) for a compact reference.

Notes
- All internal math uses quaternions for robustness; Euler angles are for display.
- We treat Earth frame as ENU (x East, y North, z Up). The robust tilt metric is the angle between the rocket nose (+X body) and Earth +Z.
- Do not change behavior in refactors without tests; see docs for layering and thread‑safety rules.

Links
- Architecture: [`docs/architecture.md`](docs/architecture.md)
- Conventions/Style: [`docs/contrib-style.md`](docs/contrib-style.md)
- Monitoring/Signals: [`docs/monitoring.md`](docs/monitoring.md), [`docs/signals.md`](docs/signals.md)
- Data Model/Telemetry: [`docs/data-model.md`](docs/data-model.md), [`docs/telemetry.md`](docs/telemetry.md)
