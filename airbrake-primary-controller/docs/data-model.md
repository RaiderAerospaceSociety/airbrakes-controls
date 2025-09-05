# Data Model And Types

Principles
- Per-device reading types: keep all values from a single sensor together.
- Telemetry snapshot selects and labels values for consumers; include a `source_id` to record origin.
- Explicit units and timestamps on all readings.

Per-device reading examples (conceptual)
- IMU1 (USFSMAX):
  - quat[4] (w,x,y,z), accel_g[3], optional gyro_dps[3] if available, valid, timestamp_ms
- IMU2 (MPU6050):
  - accel_g[3], gyro_dps[3], temp_c, valid, timestamp_ms
- BMP390:
  - pressure_pa, temperature_c, altitude_m, valid, timestamp_ms

Telemetry record provenance
- Provide a `SourceId` enum for telemetry fields that can come from different origins:
  - USFSMAX, MPU6050, BMP390, FUSED, DERIVED
- FUSED: a value produced by combining two or more sensors (e.g., variance-weighted average rate).
- DERIVED: a value computed by formula/transformation (e.g., climb rate from altitude, tilt from accel-only).

Guidelines
- Prefer completeness in early development: include fields you think you may need.
- We can prune for size/performance later once usage stabilizes.
- Always document units and any calibration/frames (body vs world, axis conventions).

