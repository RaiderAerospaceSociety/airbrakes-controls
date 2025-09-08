# Tools

## Tilt Visualizer

A minimal Python GUI for viewing the rocket's tilt angle in real time, with an optional collapsible raw serial monitor pane.

### Serial data format

The flight computer should emit newline-terminated ASCII lines like:

```
tilt_deg:<angle>
```

Where `<angle>` is a floating-point number (0–180) giving the tilt degrees off vertical. Example firmware code:

```
printf("tilt_deg:%0.2f\n", tilt_deg);
```

### Usage

1. Install dependencies:
   ```bash
   pip install pyserial matplotlib
   ```
2. Run the visualizer:
   ```bash
   python tools/tilt_visualizer.py --port COM3  # or /dev/tty.usbmodemXXXX
   ```
   Replace `COM3` with the serial port used by your microcontroller.

Extras
- Start with the raw serial monitor expanded: `--show-raw`
- Keep more/less raw lines: `--raw-buffer 500`
- Also tee raw lines to stdout: `--print-raw`

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
