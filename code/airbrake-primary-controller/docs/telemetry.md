# Telemetry Snapshot (Internal)

Definition
- Telemetry is an internal, periodic snapshot of the system state. It is not a radio downlink here.
- Built at `TELEM_PERIOD_MS` and consumed by logging, monitoring, and (later) controls.

Why
- Consistent view for all consumers regardless of individual sensor update timing.
- Single place to attach derived/fused values and health metrics.

Versioning and presence
- `include/telemetry.h` defines the record layout and `TELEM_VERSION`.
- `present_flags` (TP_*) indicate which sections were populated on a given tick.
- Bump `TELEM_VERSION` when changing layout and note changes here.

Sources
- Each field in the snapshot can record a `source_id` (USFSMAX, MPU6050, BMP390, FUSED, DERIVED) to document provenance when values can come from multiple origins.
- Per-sensor reading structs remain per-device; telemetry chooses the representative value(s) to publish each tick.

Rates
- Snapshot rate should be at least as fast as the fastest quantity you care to consume.
- Sensors may run at different rates; the snapshot always carries the latest available values with their timestamps.

