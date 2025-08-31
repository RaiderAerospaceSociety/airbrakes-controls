# Development Setup (VS Code + PlatformIO)

Requirements
- VS Code with the PlatformIO extension
- Board: Unexpected Maker FeatherS3 (ESP32-S3)

Build, Upload, Monitor
- Build: `pio run -e um_feathers3`
- Upload: `pio run -e um_feathers3 -t upload`
- Monitor (115200 baud): `pio device monitor -b 115200`

Project entry points
- `src/main.cpp:setup()` — initializes buses/logging, starts tasks
- `include/app_config.h` — task stacks/priorities/periods and monitor config (to be split into config/*)

Notes
- The first USB serial port appears after flashing; PlatformIO is configured not to wait unnecessarily for the upload port.
- If the I2C bus is wired, startup prints a one-time scan of discovered device addresses.

