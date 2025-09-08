#!/usr/bin/env python3
"""Visualize tilt angle and optionally show a raw serial monitor.

Serial data format
------------------
Expected input from the device (newline-terminated ASCII lines):

    tilt_deg:<value>

Other lines are accepted and shown in the raw monitor but ignored for plotting.

Usage::

    python tools/tilt_visualizer.py --port COM3  # or /dev/tty.usbmodemXXXX

Options:
    --show-raw     Start with the raw monitor expanded (collapsible)
    --raw-buffer   Number of raw lines to keep (default 300)
    --print-raw    Also print all raw lines to stdout

Requires: pyserial, matplotlib (pip install pyserial matplotlib)
"""

from __future__ import annotations

import argparse
from collections import deque

import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from typing import Deque
import serial
import threading


# ---------------------------------------------------------------------------
# CLI parsing


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Display tilt_deg streamed over serial")
    parser.add_argument("--port", required=True, help="Serial port, e.g. COM3 or /dev/tty.usbmodem123")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument(
        "--window", type=int, default=200, help="Number of recent samples to display"
    )
    parser.add_argument(
        "--show-raw", action="store_true", help="Start with raw serial monitor expanded"
    )
    parser.add_argument(
        "--raw-buffer", type=int, default=300, help="Max raw lines to keep in the monitor"
    )
    parser.add_argument(
        "--print-raw", action="store_true", help="Also print all raw lines to stdout"
    )
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Plotter


class TiltPlotter:
    """Real-time tilt plotter with optional raw serial monitor pane."""

    def __init__(self, ser: serial.Serial, window: int, *, show_raw: bool, raw_buffer: int, print_raw: bool) -> None:
        self.ser = ser
        self.print_raw = print_raw
        self.data: Deque[float] = deque(maxlen=window)
        self.raw_lines: Deque[str] = deque(maxlen=raw_buffer)
        self._lock = threading.Lock()
        self._stop_reader = False

        # Figure layout: main plot + optional raw monitor pane
        self.fig = plt.figure(constrained_layout=True)
        gs = GridSpec(2, 1, height_ratios=[3, 1], figure=self.fig)
        self.ax = self.fig.add_subplot(gs[0, 0])
        self.ax_raw = self.fig.add_subplot(gs[1, 0])

        # Main plot formatting
        (self.line,) = self.ax.plot([], [], lw=2)
        self.ax.set_title("Tilt (deg)")
        self.ax.set_ylabel("Tilt (deg)")
        self.ax.set_ylim(0, 180)

        # Raw monitor formatting
        self.raw_text = self.ax_raw.text(
            0.01,
            0.99,
            "",
            transform=self.ax_raw.transAxes,
            va="top",
            ha="left",
            family="monospace",
            fontsize=9,
        )
        self.ax_raw.set_title("Raw Serial Monitor (press 'r' to toggle)")
        self.ax_raw.set_axis_off()
        self.raw_visible = bool(show_raw)
        self.ax_raw.set_visible(self.raw_visible)

        # Checkbox to toggle raw monitor
        try:
            from matplotlib.widgets import CheckButtons

            cb_ax = self.fig.add_axes([0.88, 0.8, 0.1, 0.12])
            self.cb = CheckButtons(cb_ax, ["Raw"], [self.raw_visible])

            def toggle_raw():
                self.raw_visible = not self.raw_visible
                self.ax_raw.set_visible(self.raw_visible)
                self.fig.canvas.draw_idle()

            # Use the same toggle for button clicks
            self.cb.on_clicked(lambda _label: toggle_raw())
        except Exception:
            self.cb = None

        # Keyboard toggle: press 'r' to show/hide raw monitor
        def on_key(event):
            if event.key and event.key.lower() == "r":
                if self.cb is not None:
                    try:
                        # Let the CheckButton callback handle the toggle
                        self.cb.set_active(0)
                        return
                    except Exception:
                        pass
                # Fallback if no CheckButton
                self.raw_visible = not self.raw_visible
                self.ax_raw.set_visible(self.raw_visible)
                self.fig.canvas.draw_idle()

        self.fig.canvas.mpl_connect("key_press_event", on_key)

        # Stop the reader thread cleanly when the window closes
        def on_close(_event):
            self._stop_reader = True
        self.fig.canvas.mpl_connect("close_event", on_close)

        # Start a background reader thread that mirrors serial_monitor.py
        self._reader = threading.Thread(target=self._reader_loop, name="serial-reader", daemon=True)
        self._reader.start()

    # ------------------------------------------------------------------
    def _reader_loop(self) -> None:
        while not self._stop_reader:
            try:
                raw = self.ser.readline()
            except serial.SerialException:
                break
            if not raw:
                continue
            try:
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            except Exception:
                line = repr(raw)

            if self.print_raw:
                try:
                    print(line)
                except Exception:
                    pass

            with self._lock:
                self.raw_lines.append(line)
                if line.startswith("tilt_deg:"):
                    try:
                        value = float(line.split(":", 1)[1])
                        self.data.append(value)
                    except ValueError:
                        pass

    # ------------------------------------------------------------------
    def _update(self, _frame: int):
        """Update plot and raw monitor using data collected by reader thread."""
        with self._lock:
            y = list(self.data)
            raw_snapshot = list(self.raw_lines)

        # Update plot
        x = range(len(y))
        self.line.set_data(x, y)
        self.ax.set_xlim(0, max(len(y), 100))

        # Update raw monitor text (newest at top)
        if self.raw_visible:
            self.raw_text.set_text("\n".join(reversed(raw_snapshot)))

        # When blitting is disabled, return value is ignored; keep signature
        return self.line,

    # ------------------------------------------------------------------
    def run(self) -> None:
        # Disable blitting so the raw text area updates reliably across backends
        self.ani = animation.FuncAnimation(self.fig, self._update, interval=50, blit=False)
        plt.show()


# ---------------------------------------------------------------------------
# Entrypoint


def main() -> None:
    args = parse_args()
    # Open serial like serial_monitor.py does and clear any stale input
    with serial.Serial(args.port, baudrate=args.baud, timeout=1) as ser:
        try:
            ser.reset_input_buffer()
        except Exception:
            pass
        plotter = TiltPlotter(
            ser,
            args.window,
            show_raw=args.show_raw,
            raw_buffer=args.raw_buffer,
            print_raw=args.print_raw,
        )
        plotter.run()


if __name__ == "__main__":
    main()
