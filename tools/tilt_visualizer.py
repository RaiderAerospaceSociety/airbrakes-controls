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
import math

import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from typing import Deque, Dict, List, Optional, Tuple
import serial
import threading
from matplotlib.patches import Circle


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
        "--metric",
        action="append",
        default=[],
        help="Metric spec 'name[:ymin:ymax]'. Repeatable. Default: tilt_deg (autoscale)",
    )
    parser.add_argument(
        "--flag",
        action="append",
        default=[],
        help="Binary flag key to show as red/green light. Prefix with '!' to invert (true=red). Repeatable.",
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


def parse_metric_spec(spec: str) -> Tuple[str, Optional[float], Optional[float]]:
    """Parse 'name[:ymin:ymax]' into (name, ymin, ymax). Missing parts -> None."""
    parts = spec.split(":")
    if len(parts) == 1:
        return parts[0], None, None
    if len(parts) == 3:
        name, ymin_s, ymax_s = parts
        ymin = float(ymin_s) if ymin_s else None
        ymax = float(ymax_s) if ymax_s else None
        return name, ymin, ymax
    # Fallback: treat everything as name
    return spec, None, None


# ---------------------------------------------------------------------------
# Plotter


class TiltPlotter:
    """Real-time tilt plotter with optional raw serial monitor pane."""

    def __init__(
        self,
        ser: serial.Serial,
        window: int,
        *,
        show_raw: bool,
        raw_buffer: int,
        print_raw: bool,
        metric_specs: List[Tuple[str, Optional[float], Optional[float]]],
        flags: List[str],
    ) -> None:
        self.ser = ser
        self.print_raw = print_raw
        self.window = window
        # Metrics: config and data buffers
        self.metric_cfgs: List[Tuple[str, Optional[float], Optional[float]]] = metric_specs
        self.metric_data: Dict[str, Deque[float]] = {name: deque(maxlen=window) for name, _, _ in self.metric_cfgs}
        # Shared timeline (seconds since first seen ts), aligned across metrics
        self.ts: Deque[float] = deque(maxlen=window)
        self._t0_ms: Optional[float] = None
        self._dt_guess: float = 0.05  # fallback step if no ts provided
        # Flags: config (with invert) and state storage
        self.flag_cfgs: List[Tuple[str, bool]] = []  # (name, invert)
        for f in flags:
            invert = f.startswith("!")
            name = f[1:] if invert else f
            if not name:
                continue
            self.flag_cfgs.append((name, invert))
        self.flag_state: Dict[str, Optional[bool]] = {name: None for name, _inv in self.flag_cfgs}
        self.raw_lines: Deque[str] = deque(maxlen=raw_buffer)
        self._lock = threading.Lock()
        self._stop_reader = False

        # Figure layout: main plot + optional raw monitor pane
        self.fig = plt.figure(constrained_layout=True)
        # Layout: metrics (N rows) + flags (1 row if any) + raw (1 row)
        n_metrics = max(1, len(self.metric_cfgs))
        has_flags = len(self.flag_cfgs) > 0
        rows = n_metrics + (1 if has_flags else 0) + 1
        heights: List[int] = [3] * n_metrics + ([1] if has_flags else []) + [1]
        gs = GridSpec(rows, 1, height_ratios=heights, figure=self.fig)
        # Metric axes
        self.metric_axes: Dict[str, plt.Axes] = {}
        self.metric_lines: Dict[str, any] = {}
        for idx, (name, ymin, ymax) in enumerate(self.metric_cfgs or [("tilt_deg", None, None)]):
            ax = self.fig.add_subplot(gs[idx, 0])
            (line,) = ax.plot([], [], lw=2, label=name)
            ax.set_title(name)
            ax.set_ylabel(name)
            if ymin is not None and ymax is not None:
                ax.set_ylim(ymin, ymax)
            self.metric_axes[name] = ax
            self.metric_lines[name] = line
        # Flags axis (optional)
        self.ax_flags: Optional[plt.Axes] = None
        self.flag_artists: Dict[str, Tuple[Circle, any]] = {}
        if has_flags:
            flags_row = n_metrics
            self.ax_flags = self.fig.add_subplot(gs[flags_row, 0])
            self.ax_flags.set_axis_off()
            self._init_flags_artists()
        # Raw monitor axis is last row
        self.ax_raw = self.fig.add_subplot(gs[rows - 1, 0])

        # Main plot formatting for metric axes handled above

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
    def _init_flags_artists(self) -> None:
        assert self.ax_flags is not None
        n = len(self.flag_cfgs)
        # Arrange lights horizontally
        spacing = 1.0
        xs = [i * spacing for i in range(n)]
        for i, (name, invert) in enumerate(self.flag_cfgs):
            x = xs[i]
            # Draw a light (circle) and a label to the right
            circ = Circle((x, 0), 0.25, facecolor="#cccccc", edgecolor="#333333")
            self.ax_flags.add_patch(circ)
            label = self.ax_flags.text(x + 0.4, 0, name, va="center", ha="left", fontsize=10)
            self.flag_artists[name] = (circ, label)
        # Configure axis limits and hide spines/ticks
        self.ax_flags.set_xlim(-0.5, xs[-1] + 1.0 if xs else 1.0)
        self.ax_flags.set_ylim(-0.6, 0.6)
        self.ax_flags.set_axis_off()

    def _update_flag_colors(self) -> None:
        # Green for True, Red for False, Gray for None
        invert_map = {name: inv for name, inv in self.flag_cfgs}
        for name, (circ, _label) in self.flag_artists.items():
            state = self.flag_state.get(name)
            inv = invert_map.get(name, False)
            if state is None:
                circ.set_facecolor("#cccccc")  # unknown/gray
            else:
                show_red = (state is True and inv) or (state is False and not inv)
                if show_red:
                    circ.set_facecolor("#d62728")  # red
                else:
                    circ.set_facecolor("#2ca02c")  # green

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

            # Parse one or more key:value pairs (comma-separated accepted)
            chunks = [c.strip() for c in line.split(",") if c.strip()]
            line_vals: Dict[str, float] = {}
            ts_ms: Optional[float] = None
            for ch in chunks:
                if ":" not in ch:
                    continue
                key, value_s = ch.split(":", 1)
                key = key.strip()
                # Strip Teleplot leading '>' if present
                if key.startswith(">"):
                    key = key[1:].strip()
                value_s = value_s.strip()

                if key == "ts":
                    try:
                        ts_ms = float(value_s)
                    except ValueError:
                        ts_ms = None
                    continue

                # Timeseries metrics
                if key in self.metric_data:
                    try:
                        value = float(value_s)
                        line_vals[key] = value
                    except ValueError:
                        pass
                # Binary flags (truthy -> True)
                if key in self.flag_state:
                    try:
                        v = float(value_s)
                        state = (v != 0)
                    except ValueError:
                        # Accept textual true/false
                        val_low = value_s.lower()
                        if val_low in ("true", "on", "high", "set"):
                            state = True
                        elif val_low in ("false", "off", "low", "clear"):
                            state = False
                        else:
                            continue
                    self.flag_state[key] = state

            with self._lock:
                self.raw_lines.append(line)
                # Update shared timeline
                if ts_ms is not None:
                    if self._t0_ms is None:
                        self._t0_ms = ts_ms
                    t_sec = max(0.0, (ts_ms - self._t0_ms) / 1000.0)
                else:
                    # Fallback: synthetic time
                    if self.ts:
                        t_sec = self.ts[-1] + self._dt_guess
                    else:
                        t_sec = 0.0
                self.ts.append(t_sec)

                # For each metric, append new value or NaN placeholder
                for name, _ymin, _ymax in self.metric_cfgs:
                    buf = self.metric_data.get(name)
                    if buf is None:
                        continue
                    val = line_vals.get(name, float("nan"))
                    buf.append(val)

    # ------------------------------------------------------------------
    def _update(self, _frame: int):
        """Update plot and raw monitor using data collected by reader thread."""
        with self._lock:
            metric_snapshot = {k: list(v) for k, v in self.metric_data.items()}
            ts_snapshot = list(self.ts)
            raw_snapshot = list(self.raw_lines)
            flag_snapshot = dict(self.flag_state)

        # Update metric plots
        for name, data in metric_snapshot.items():
            ax = self.metric_axes.get(name)
            line = self.metric_lines.get(name)
            if ax is None or line is None:
                continue
            if ts_snapshot and len(ts_snapshot) == len(data):
                x = ts_snapshot
            else:
                x = list(range(len(data)))
            line.set_data(x, data)
            # X window
            if ts_snapshot:
                xmin = ts_snapshot[0]
                xmax = ts_snapshot[-1] if ts_snapshot[-1] > xmin else xmin + 1.0
                ax.set_xlim(xmin, xmax)
            else:
                ax.set_xlim(0, max(len(data), self.window))
            # Autoscale Y if not fixed
            _, ymin, ymax = next((cfg for cfg in self.metric_cfgs if cfg[0] == name), (name, None, None))
            if ymin is None or ymax is None:
                try:
                    ax.relim()
                    ax.autoscale_view(scalex=False, scaley=True)
                except Exception:
                    pass

        # Update raw monitor text (newest at top)
        if self.raw_visible:
            self.raw_text.set_text("\n".join(reversed(raw_snapshot)))

        # Update flags
        if self.ax_flags is not None:
            self.flag_state.update(flag_snapshot)
            self._update_flag_colors()

        # When blitting is disabled, return value is ignored; return metric artists
        return tuple(self.metric_lines.values())

    # ------------------------------------------------------------------
    def run(self) -> None:
        # Disable blitting so the raw text area updates reliably across backends
        self.ani = animation.FuncAnimation(self.fig, self._update, interval=50, blit=False)
        plt.show()


# ---------------------------------------------------------------------------
# Entrypoint


def main() -> None:
    args = parse_args()
    # Prepare metric and flag configurations
    metric_specs: List[Tuple[str, Optional[float], Optional[float]]] = (
        [parse_metric_spec(s) for s in args.metric] if args.metric else [("tilt_deg", None, None)]
    )
    # Default flags: green when ok, red when lock
    flags: List[str] = args.flag or ["tilt_ok", "!tilt_lock"]

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
            metric_specs=metric_specs,
            flags=flags,
        )
        plotter.run()


if __name__ == "__main__":
    main()
