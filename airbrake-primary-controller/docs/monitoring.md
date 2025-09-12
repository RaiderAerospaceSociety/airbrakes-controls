# Monitoring And Debug Output

Goals
- Quickly visualize values during development.
- Compare multiple sensors that measure the same quantity (e.g., IMU accelerometers).
- Keep configuration simple and consistent.

Current modes
- VS Code Serial Plotter format: lines begin with `>` and contain `label:value` pairs
  - Example: `>imu1_ax_g:0.0123,imu2_ax_g:0.0101,diff_ax_g:0.0022`
  - Configure in `include/app_config.h` via `SERIAL_PLOTTER_MODE` and PLOT_* macros.
- CSV mode (optional): comma-separated values with header rows (currently behind compile-time toggles).

Flight visualizer (tools)
- A Python dashboard exists at `tools/flight_visualizer.py` to monitor real-time telemetry (battery, airbrakes, state, lights, graphs, compass).
- It includes a collapsible Raw Serial Monitor to inspect all incoming lines:
  - Launch with `--show-raw` to start expanded; press `p` to toggle while running.
  - Options: `--raw-buffer N` to control retained lines, `--print-raw` to also tee to stdout.
  - Example: `python tools/flight_visualizer.py --port /dev/tty.usbmodem1101 --show-raw`.

Planned improvements
- Channel registry with presets and tags, so a single config selects a group of channels (e.g., `imu_compare`, `altitude_all`, `fused_debug`).
- Logger reads from the telemetry snapshot by default, so all consumers see a consistent view.

Tips
- Use the Serial Plotter extension’s variable list to turn channels on/off and overlay plots.
- For static tests, capture logs to file and analyze in a notebook or spreadsheet.
