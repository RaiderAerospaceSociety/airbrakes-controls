Live Signals And Plot Labels

Altitude / AGL
- `bmp1_alt_m` — BMP390 altitude (m, MSL, Earth).
- `imu1_baro_alt_m` — IMU1 internal baro altitude (m, MSL, Earth).
- `agl_bmp1_m` — AGL from BMP1 (m).
- `agl_imu1_m` — AGL from IMU1 (m).
- `agl_fused_m` — fused AGL (m).

Vertical Kinematics
- `vz_baro_mps` — vertical speed from AGL derivative (m/s, Earth +Z up).
- `vz_acc_mps` — vertical speed from accel integration (m/s, experimental).
- `vz_fused_mps` — fused vertical speed (m/s).
- `az_imu1_mps2` — vertical acceleration (m/s^2) from IMU1 in Earth frame.

Atmospherics and Mach
- `temp_C` — temperature from BMP390 (°C).
- `press_hPa` — pressure from BMP390 (hPa).
- `sos_mps` — dynamic speed of sound from temperature (m/s).
- `sos_min_mps` — conservative SoS used for gating (m/s).
- `mach_vz` — |vz_baro_mps| / sos_mps (unitless).
- `mach_cons` — conservative Mach proxy (unitless).

Attitude (display)
- `yaw_deg`, `pitch_deg`, `roll_deg` — derived from IMU1 quaternion (deg).
- `tilt_deg` — angle from vertical (deg), robust near vertical.
- `tilt_az_deg360` — azimuth of tilt direction (deg, 0..360, East=0°, North=90°).

Apogee helpers
- `t_apogee_s` — biased‑early time to apogee (s).
- `apogee_agl_m` — biased‑low predicted apogee AGL (m).

Notes
- Lines are emitted as `>label:value,label:value` at `LOGGER_PERIOD_MS`.
- All values are snapshots from the fusion service and/or latest sensor readings.
