# USFSMAX Notes

What it provides
- Quaternion and Euler (from its on-board fusion)
- Accelerometer (g), gyroscope (dps), magnetometer (uT), baro depending on configuration

Project usage
- Reader task: `src/sensor_imu1.cpp`
- Public API: `include/sensor_imu1.h`
- I2C via vendor wrapper (lib/usfsmax)

Considerations
- Treat USFSMAX quaternion as primary attitude reference.
- Avoid double-counting its internal accel/mag if you also fuse elsewhere.
- Document the body-frame axes and any rotations relative to the board.
