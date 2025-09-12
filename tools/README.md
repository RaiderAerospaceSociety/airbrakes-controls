# Tools

## Flight Visualizer

A Python dashboard for viewing key flight telemetry in real time: gauges (battery, airbrakes), status lights (sensor OKs, events), digital clocks (time alive/since liftoff/to apogee), graphs (AGL, vertical velocity, acceleration), compass for azimuth + tilt, and a collapsible raw serial monitor pane.

### Serial data format

The flight computer should emit newline-terminated ASCII lines containing comma-separated `key:value` pairs, e.g.:

```
ts_ms:40364, vbat_v:4.119, i2c_errs:0, spi_errs:0, fc_state_str:LOCKED, tilt_deg:1.14, tilt_az_deg360:90,
agl_fused_m:0.205, vz_fused_mps:0.093, az_imu1_mps2:0.106, cmd_deg:0.00, act_deg:0.00, agl_ready:1, ...
```

### Usage

1. Install dependencies:
   ```bash
   pip install pyserial matplotlib
   ```
2. Run the visualizer:
   ```bash
   python tools/flight_visualizer.py --port COM3  # or /dev/tty.usbmodemXXXX
   ```
   Replace `COM3` with the serial port used by your microcontroller.

Extras
- Start with the raw serial monitor expanded: `--show-raw`
- Keep more/less raw lines: `--raw-buffer 500`
- Also tee raw lines to stdout: `--print-raw`

Note: the previous `tools/tilt_visualizer.py` now forwards to `flight_visualizer.py`.

## Simple Serial Monitor

A tiny, dependency-light serial monitor for quick testing. It just opens a port and prints each incoming line.

### Usage

1. Install dependency:
   ```bash
   pip install pyserial
   ```
2. List ports or start monitoring:
   ```bash
   # List available ports
   python tools/serial_monitor.py --list

   # Monitor a specific port (replace with your device)
   python tools/serial_monitor.py --port COM3
   # or
   python tools/serial_monitor.py --port /dev/tty.usbmodemXXXX --baud 115200
   ```

Extras
- Add timestamps: `--timestamp`
