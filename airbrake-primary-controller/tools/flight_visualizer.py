#!/usr/bin/env python3
"""Flight visualizer: dashboard for flight computer telemetry.

Features
- Battery voltage gauge (1S Li‑ion 18650: 3.0–4.2 V).
- Numeric counters: I2C/SPI errors.
- Flight state text (fc_state_str) with lockout indicator.
- Status lights: sens_*_ok, baro_agree, mach_ok, tilt_ok, tilt_latch, liftoff_det, burnout_det, agl_ready, lockout.
- Clocks: time alive (from ts_ms), time since liftoff, time to apogee.
- Airbrake gauges: cmd_deg vs act_deg (dual-needle dial).
- Graphs: agl_fused_m, vz_fused_mps, az_imu1_mps2.
- Compass: tilt_az_deg360 with numeric tilt_deg.
- Collapsible raw serial monitor.

Input format (newline-terminated, comma-separated key:value pairs), example:
    ts_ms:40364, vbat_v:4.119, i2c_errs:0, spi_errs:0, fc_state_str:LOCKED, ...

Usage:
    python tools/flight_visualizer.py --port COM3  # or /dev/tty.usbmodemXXXX

Requires: pyserial, matplotlib (pip install pyserial matplotlib)
"""

from __future__ import annotations

import argparse
from collections import deque
import math
import threading
from typing import Deque, Dict, List, Optional, Tuple

import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec, GridSpecFromSubplotSpec
from matplotlib.patches import Circle
import numpy as np
import serial


# Apply dark theme defaults early
try:
    plt.style.use('dark_background')
except Exception:
    pass

# Custom dark palette
FIG_BG = "#0c0f14"
AX_BG = "#12161c"
GAUGE_BG = "#171c24"
GAUGE_BAR_BG = "#2b3442"
UNKNOWN_GREY = "#666666"

# Override relevant rcParams
plt.rcParams.update({
    'figure.facecolor': FIG_BG,
    'axes.facecolor': AX_BG,
    'savefig.facecolor': FIG_BG,
    'text.color': '#e6e6e6',
    'axes.labelcolor': '#e6e6e6',
    'axes.edgecolor': '#888888',
    'xtick.color': '#cccccc',
    'ytick.color': '#cccccc',
    'grid.color': '#444444',
})

# ---------------------------------------------------------------------------
# CLI


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Flight visualizer dashboard")
    p.add_argument("--port", required=True, help="Serial port, e.g. COM3 or /dev/tty.usbmodem123")
    p.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    p.add_argument("--window", type=int, default=300, help="Timeseries window (samples)")
    p.add_argument("--show-raw", action="store_true", help="Start with raw monitor expanded")
    p.add_argument("--raw-buffer", type=int, default=400, help="Raw lines kept in monitor")
    p.add_argument("--print-raw", action="store_true", help="Also print all raw lines to stdout")
    p.add_argument("--fps", type=int, default=20, help="UI update rate (frames per second)")
    return p.parse_args()


# ---------------------------------------------------------------------------
# Small UI helpers


def clamp(v: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(vmax, v))


def fmt_time(s: Optional[float]) -> str:
    if s is None or math.isnan(s):
        return "--:--.--"
    s = max(0.0, float(s))
    m = int(s // 60)
    rem = s - m * 60
    return f"{m:02d}:{rem:05.2f}"


class HBarGauge:
    """Simple horizontal bar gauge with numeric label."""

    def __init__(self, ax: plt.Axes, vmin: float, vmax: float, label: str, unit: str = "") -> None:
        self.ax = ax
        self.vmin = float(vmin)
        self.vmax = float(vmax)
        self.unit = unit
        # Configure axis
        ax.set_xlim(self.vmin, self.vmax)
        ax.set_ylim(0, 1)
        ax.set_yticks([])
        ax.set_xlabel(label)
        ax.set_facecolor(GAUGE_BG)
        for spine in ax.spines.values():
            spine.set_visible(False)
        # Background bar
        self.bg = ax.barh(0.5, width=(self.vmax - self.vmin), left=self.vmin, height=0.6, color=GAUGE_BAR_BG)
        # Foreground (value) bar
        self.fg = ax.barh(0.5, width=0.0, left=self.vmin, height=0.6, color="#2ca02c")
        # Text centered
        self.txt = ax.text(0.5, 0.5, "", transform=ax.transAxes, va="center", ha="center", fontsize=12, fontweight="bold")

    def _color_for_fraction(self, frac: float) -> str:
        # Red (low) -> Yellow -> Green (high)
        frac = clamp(frac, 0.0, 1.0)
        if frac < 0.2:
            return "#d62728"
        if frac < 0.4:
            return "#ff7f0e"
        if frac < 0.7:
            return "#ffbf00"
        return "#2ca02c"

    def update(self, value: Optional[float]) -> None:
        if value is None or math.isnan(value):
            self.fg[0].set_width(0.0)
            self.fg[0].set_color(UNKNOWN_GREY)
            self.txt.set_text("--")
            return
        value = float(value)
        value = clamp(value, self.vmin, self.vmax)
        frac = (value - self.vmin) / (self.vmax - self.vmin) if self.vmax > self.vmin else 0.0
        self.fg[0].set_width(value - self.vmin)
        self.fg[0].set_color(self._color_for_fraction(frac))
        self.txt.set_text(f"{value:0.3f}{self.unit}")


class DialGaugeDual:
    """Simple semicircular dial with two needles (e.g., cmd/act degrees)."""

    def __init__(self, ax: plt.Axes, vmin: float, vmax: float, label: str, n_major: int = 6) -> None:
        self.ax = ax
        self.vmin = float(vmin)
        self.vmax = float(vmax)
        self.label = label
        ax.set_aspect('equal')
        ax.set_axis_off()
        # Dial background
        theta = np.linspace(-120, 120, 241) * np.pi / 180.0
        r = np.ones_like(theta)
        ax.plot(r * np.cos(theta), r * np.sin(theta), color="#aaaaaa", lw=2)
        # Ticks and labels
        for i in range(n_major + 1):
            v = self.vmin + i * (self.vmax - self.vmin) / n_major
            ang = self._val_to_angle(v)
            x0, y0 = np.cos(ang), np.sin(ang)
            ax.plot([0.88 * x0, 1.0 * x0], [0.88 * y0, 1.0 * y0], color="#999999", lw=2)
            tx = 0.72 * x0
            ty = 0.72 * y0
            ax.text(tx, ty, f"{v:.0f}", ha="center", va="center", fontsize=9)
        # Center and label
        self.center = ax.plot(0, 0, 'o', color="#dddddd")[0]
        ax.text(0, -0.35, label, ha="center", va="center", fontsize=11, fontweight="bold")
        # Needles
        (self.n_cmd_line,) = ax.plot([0, 0], [0, 0], color="#1f77b4", lw=3, label="cmd")
        (self.n_act_line,) = ax.plot([0, 0], [0, 0], color="#d62728", lw=3, label="act")
        # Legend-ish text
        ax.text(-0.95, 1.05, "cmd", color="#1f77b4", fontsize=9)
        ax.text(-0.60, 1.05, "act", color="#d62728", fontsize=9)
        # Value text
        self.txt = ax.text(0, 0.15, "", ha="center", va="center", fontsize=12, fontweight="bold")

    def _val_to_angle(self, v: float) -> float:
        v = clamp(v, self.vmin, self.vmax)
        frac = (v - self.vmin) / (self.vmax - self.vmin) if self.vmax > self.vmin else 0.0
        deg = -120.0 + frac * 240.0
        return deg * np.pi / 180.0

    def update(self, cmd: Optional[float], act: Optional[float]) -> None:
        for val, line in ((cmd, self.n_cmd_line), (act, self.n_act_line)):
            if val is None or math.isnan(val):
                line.set_data([0, 0], [0, 0])
            else:
                ang = self._val_to_angle(float(val))
                line.set_data([0, np.cos(ang)], [0, np.sin(ang)])
        # Center numeric text: show both values if available
        txt = []
        if cmd is not None and not math.isnan(cmd):
            txt.append(f"cmd {cmd:.1f}°")
        if act is not None and not math.isnan(act):
            txt.append(f"act {act:.1f}°")
        self.txt.set_text("  |  ".join(txt))


class Compass:
    """Compass showing azimuth (deg 0-360, 0=N) and tilt magnitude as text."""

    def __init__(self, ax: plt.Axes, label: str = "Azimuth/ Tilt") -> None:
        self.ax = ax
        ax.set_aspect('equal')
        ax.set_axis_off()
        # Circle and cardinal directions
        theta = np.linspace(0, 2 * np.pi, 360)
        ax.plot(np.cos(theta), np.sin(theta), color="#aaaaaa", lw=2)
        # Cardinal labels
        for ang_deg, lab in ((0, "E"), (90, "N"), (180, "W"), (270, "S")):
            ang = np.deg2rad(ang_deg)
            ax.text(1.08 * np.cos(ang), 1.08 * np.sin(ang), lab, ha="center", va="center", fontsize=10)
        # Heading needle
        (self.heading_line,) = ax.plot([0, 0], [0, 1], color="#1f77b4", lw=3)
        self.txt = ax.text(0, -1.2, label, ha="center", va="center", fontsize=11, fontweight="bold")
        self.center_txt = ax.text(0, 0, "", ha="center", va="center", fontsize=13, fontweight="bold")

    @staticmethod
    def _azi_to_angle_rad(azi_deg: float) -> float:
        # Convert compass degrees (0=N, clockwise) to math angle (0=+x, CCW)
        # In our axis with 0° label at E, rotate: math_theta = 90° - azi
        return np.deg2rad(90.0 - azi_deg)

    def update(self, azi_deg: Optional[float], tilt_deg: Optional[float]) -> None:
        if azi_deg is None or math.isnan(azi_deg):
            self.heading_line.set_data([0, 0], [0, 0])
        else:
            ang = self._azi_to_angle_rad(float(azi_deg))
            self.heading_line.set_data([0, np.cos(ang)], [0, np.sin(ang)])
        if tilt_deg is None or math.isnan(tilt_deg):
            self.center_txt.set_text("tilt: --°")
        else:
            self.center_txt.set_text(f"tilt: {float(tilt_deg):.2f}°")


class LightsPanel:
    """Horizontal row of status lights with labels."""

    def __init__(self, ax: plt.Axes, names: List[str]) -> None:
        self.ax = ax
        self.names = names
        self.circles: Dict[str, Circle] = {}
        self._init()

    def _init(self) -> None:
        ax = self.ax
        ax.set_axis_off()
        spacing = 1.5
        xs = [i * spacing for i in range(len(self.names))]
        for i, name in enumerate(self.names):
            x = xs[i]
            circ = Circle((x, 0), 0.25, facecolor="#555555", edgecolor="#aaaaaa")
            ax.add_patch(circ)
            ax.text(x + 0.4, 0, name, va="center", ha="left", fontsize=10)
            self.circles[name] = circ
        ax.set_xlim(-0.5, (xs[-1] + 1.0) if xs else 1.0)
        ax.set_ylim(-0.6, 0.6)

    def update(self, states: Dict[str, Optional[bool]]) -> None:
        for name, circ in self.circles.items():
            st = states.get(name)
            if st is None:
                circ.set_facecolor("#555555")
            else:
                circ.set_facecolor("#2ca02c" if st else "#d62728")


# ---------------------------------------------------------------------------
# Main Visualizer


class FlightVisualizer:
    def __init__(self, ser: serial.Serial, window: int, *, show_raw: bool, raw_buffer: int, print_raw: bool) -> None:
        self.ser = ser
        self.window = int(window)
        self.print_raw = bool(print_raw)
        self.raw_lines: Deque[str] = deque(maxlen=int(raw_buffer))
        self._lock = threading.Lock()
        self._stop = False
        self._frame = 0
        self._raw_update_every = 5  # update raw text every N frames
        self._autoscale_every = 10  # autoscale Y every N frames
        self._prev_ts_len = 0
        self._text_cache: Dict[int, str] = {}

        # Time base
        self.ts: Deque[float] = deque(maxlen=self.window)  # seconds since first ts_ms
        self._t0_ms: Optional[float] = None
        self._dt_guess = 0.05

        # Last values (single-value fields)
        self.last: Dict[str, object] = {}

        # Flags
        #SECTION - Flags to show
        self.flag_names = [
            "sens_imu1_ok",
            "sens_bmp1_ok",
            "sens_imu2_ok",
            "baro_agree",
            "mach_ok",
            "tilt_ok",
            "tilt_latch",
            "liftoff_det",
            "burnout_det",
            "agl_ready",
            "lockout",  # computed from state if not present
        ]
        self.flags: Dict[str, Optional[bool]] = {k: None for k in self.flag_names}
        #!SECTION - Flags to show

        # Timeseries buffers
        #SECTION - Metrics to plot
        self.ts_metrics = [
            ("agl_fused_m", None, None),
            ("vz_fused_mps", None, None),
            ("az_imu1_mps2", None, None),
        ]
        self.metric_data: Dict[str, Deque[float]] = {k: deque(maxlen=self.window) for k, _, _ in self.ts_metrics}
        #!SECTION - Metrics to plot

        # Figure and layout
        self.fig = plt.figure(constrained_layout=True, figsize=(13, 9))
        # Layout rows: top, lights, dials (airbrake+compass), timeseries, spacer, raw
        rows = 6
        gs = GridSpec(rows, 1, height_ratios=[1.2, 0.9, 1.8, 2.4, 0.2, 1.0], figure=self.fig)

        # Top row: 4 columns: battery, state, clocks, errors
        gs_top = GridSpecFromSubplotSpec(1, 5, subplot_spec=gs[0, 0], wspace=0.35, width_ratios=[1.2, 1.2, 1.2, 1.0, 1.0])
        self.ax_batt = self.fig.add_subplot(gs_top[0, 0])
        self.ax_temp = self.fig.add_subplot(gs_top[0, 1])
        self.ax_state = self.fig.add_subplot(gs_top[0, 2])
        self.ax_clocks = self.fig.add_subplot(gs_top[0, 3])
        self.ax_errors = self.fig.add_subplot(gs_top[0, 4])

        # Lights row
        self.ax_lights = self.fig.add_subplot(gs[1, 0])

        # Dials row: Airbrake + Compass side-by-side
        gs_dials = GridSpecFromSubplotSpec(1, 2, subplot_spec=gs[2, 0], wspace=0.3)
        self.ax_brake = self.fig.add_subplot(gs_dials[0, 0])

        # Timeseries row (3 columns)
        gs_ts = GridSpecFromSubplotSpec(1, 3, subplot_spec=gs[3, 0], wspace=0.3)
        self.ax_ts: Dict[str, plt.Axes] = {}
        self.lines: Dict[str, any] = {}
        for i, (name, ymin, ymax) in enumerate(self.ts_metrics):
            ax = self.fig.add_subplot(gs_ts[0, i])
            (line,) = ax.plot([], [], lw=2, label=name)
            ax.set_title(name)
            ax.set_ylabel(name)
            if ymin is not None and ymax is not None:
                ax.set_ylim(ymin, ymax)
            self.ax_ts[name] = ax
            self.lines[name] = line

        # Compass on the right of the dials row
        self.ax_compass = self.fig.add_subplot(gs_dials[0, 1])

        # Spacer row (gs[4])

        # Raw row (last row index 5)
        self.ax_raw = self.fig.add_subplot(gs[5, 0])

        # Widgets/objects
        # Battery gauge scaled for 1S 18650 (typical usable range ~3.0–4.2 V)
        self.batt_gauge = HBarGauge(self.ax_batt, 3.0, 4.2, label="Battery (V)", unit="V")
        # Temperature gauge in Fahrenheit (converted from temp_c)
        self.temp_gauge = HBarGauge(self.ax_temp, 14.0, 176.0, label="Temp (F)", unit="°F")

        # State panel
        self.ax_state.set_axis_off()
        self.state_txt = self.ax_state.text(0.02, 0.65, "STATE: --", transform=self.ax_state.transAxes, ha="left", va="center", fontsize=13, fontweight="bold")
        self.lockout_txt = self.ax_state.text(0.02, 0.35, "LOCKOUT: --", transform=self.ax_state.transAxes, ha="left", va="center", fontsize=12)

        # Clocks
        self.ax_clocks.set_axis_off()
        self.clock_alive = self.ax_clocks.text(0.02, 0.70, "Alive: --:--.--", transform=self.ax_clocks.transAxes, ha="left", va="center", fontsize=12)
        self.clock_since = self.ax_clocks.text(0.02, 0.45, "Since liftoff: --:--.--", transform=self.ax_clocks.transAxes, ha="left", va="center", fontsize=12)
        self.clock_to_ap = self.ax_clocks.text(0.02, 0.20, "To apogee: --:--.--", transform=self.ax_clocks.transAxes, ha="left", va="center", fontsize=12)

        # Errors
        self.ax_errors.set_axis_off()
        self.err_i2c = self.ax_errors.text(0.02, 0.65, "i2c_errs: --", transform=self.ax_errors.transAxes, ha="left", va="center", fontsize=12)
        self.err_spi = self.ax_errors.text(0.02, 0.35, "spi_errs: --", transform=self.ax_errors.transAxes, ha="left", va="center", fontsize=12)

        # Lights panel
        self.lights = LightsPanel(self.ax_lights, self.flag_names)

        # Airbrake gauge (0–90° typical)
        self.brake_dial = DialGaugeDual(self.ax_brake, vmin=0.0, vmax=90.0, label="Airbrake deg")

        # Compass
        self.compass = Compass(self.ax_compass, label="Azimuth / Tilt")

        # Raw monitor
        self.raw_text = self.ax_raw.text(0.01, 0.99, "", transform=self.ax_raw.transAxes, va="top", ha="left", family="monospace", fontsize=9)
        self.ax_raw.set_title("Raw Serial Monitor (press 'p' to toggle)")
        self.ax_raw.set_axis_off()
        self.raw_visible = bool(show_raw)
        self.ax_raw.set_visible(self.raw_visible)

        # Keyboard toggle for raw
        def on_key(event):
            if not event.key:
                return
            k = event.key.lower()
            if k == "p":
                # Toggle raw panel visibility
                self.raw_visible = not self.raw_visible
                self.ax_raw.set_visible(self.raw_visible)
                self.fig.canvas.draw_idle()
            elif k == "r":
                # Reload serial connection to handle device resets
                self._reload_serial()

        self.fig.canvas.mpl_connect("key_press_event", on_key)

        def on_close(_e):
            self._stop = True
            try:
                self.ser.close()
            except Exception:
                pass

        self.fig.canvas.mpl_connect("close_event", on_close)

        # Reader thread
        self._reader_id = 0
        try:
            self._port = getattr(ser, 'port', None)
            self._baud = getattr(ser, 'baudrate', 115200)
        except Exception:
            self._port, self._baud = None, 115200
        self._reader = threading.Thread(target=self._reader_loop, args=(self._reader_id,), name="serial-reader", daemon=True)
        self._reader.start()

    # ----------------------------- Reader ---------------------------------
    def _reader_loop(self, my_id: int) -> None:
        while not self._stop and my_id == getattr(self, '_reader_id', -1):
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

            # Parse as comma-separated key:value pairs; accept teleplot leading '>'
            parts = [p.strip() for p in line.split(",") if p.strip()]
            ts_ms: Optional[float] = None
            values: Dict[str, object] = {}
            for ch in parts:
                if ":" not in ch:
                    continue
                k, vs = ch.split(":", 1)
                k = k.strip()
                if k.startswith(">"):
                    k = k[1:].strip()
                vs = vs.strip()
                # Timestamp keys
                if k in ("ts_ms", "ts"):
                    try:
                        ts_ms = float(vs)
                    except ValueError:
                        ts_ms = None
                    continue
                # String channels (state)
                if k == "fc_state_str":
                    values[k] = vs
                    continue
                # Try float
                try:
                    v = float(vs)
                    values[k] = v
                except ValueError:
                    # Ignore non-float unexpected strings
                    values[k] = vs

            with self._lock:
                self.raw_lines.append(line)

                # Update timebase
                if ts_ms is not None:
                    if self._t0_ms is None:
                        self._t0_ms = ts_ms
                    t = max(0.0, (ts_ms - self._t0_ms) / 1000.0)
                else:
                    t = (self.ts[-1] + self._dt_guess) if self.ts else 0.0
                self.ts.append(t)

                # Last values and flags
                self.last.update(values)

                # Derived lockout flag from state if not present
                st = str(self.last.get("fc_state_str", ""))
                if "lockout" not in values:
                    self.flags["lockout"] = True if st.upper() == "ABORT_LOCKOUT" else False if st else None

                # Update explicit boolean-like flags if present
                for name in self.flag_names:
                    if name in values:
                        try:
                            self.flags[name] = (float(values[name]) != 0.0)
                        except Exception:
                            # non-numeric -> ignore
                            pass

                # Update timeseries
                for name in self.metric_data.keys():
                    v = values.get(name)
                    try:
                        vf = float(v) if v is not None else float('nan')
                    except Exception:
                        vf = float('nan')
                    self.metric_data[name].append(vf)

    # ----------------------------- Update ---------------------------------
    def _update(self, _frame: int):
        with self._lock:
            ts = list(self.ts)
            metric_snapshot = {k: list(v) for k, v in self.metric_data.items()}
            last = dict(self.last)
            flags = dict(self.flags)
            raw = list(self.raw_lines)

        # Battery
        self.batt_gauge.update(self._get_float(last.get("vbat_v")))

        # Temperature (convert C->F)
        tc = self._get_float(last.get("temp_c"))
        tf = (tc * 9.0 / 5.0 + 32.0) if tc is not None else None
        self.temp_gauge.update(tf)

        # State and lockout
        st = str(last.get("fc_state_str", "")) if last.get("fc_state_str") is not None else ""
        state_txt = st if st else "--"
        self._set_text(self.state_txt, f"STATE: {state_txt}")
        lockout = flags.get("lockout")
        self._set_text(self.lockout_txt, f"LOCKOUT: {'ON' if lockout else ('OFF' if lockout is not None else '--')}")
        self.lockout_txt.set_color("#d62728" if lockout else ("#2ca02c" if lockout is not None else "#bbbbbb"))

        # Clocks
        t_alive = ts[-1] if ts else None
        t_since = self._get_float(last.get("t_since_launch_s"))
        t_to_ap = self._get_float(last.get("t_to_apogee_s"))
        self._set_text(self.clock_alive, f"Alive: {fmt_time(t_alive)}")
        self._set_text(self.clock_since, f"Since liftoff: {fmt_time(t_since)}")
        self._set_text(self.clock_to_ap, f"To apogee: {fmt_time(t_to_ap)}")

        # Errors
        i2c_errs = self._get_int(last.get("i2c_errs"))
        spi_errs = self._get_int(last.get("spi_errs"))
        self._set_text(self.err_i2c, f"i2c_errs: {i2c_errs if i2c_errs is not None else '--'}")
        self._set_text(self.err_spi, f"spi_errs: {spi_errs if spi_errs is not None else '--'}")

        # Flags
        self.lights.update(flags)

        # Airbrakes
        cmd = self._get_float(last.get("cmd_deg"))
        act = self._get_float(last.get("act_deg"))
        self.brake_dial.update(cmd, act)

        # Timeseries plots
        for name, data in metric_snapshot.items():
            line = self.lines.get(name)
            ax = self.ax_ts.get(name)
            if line is None or ax is None:
                continue
            if ts and len(ts) == len(data):
                x = ts
            else:
                x = list(range(len(data)))
            line.set_data(x, data)
            # X window only when ts length changed
            if len(ts) != self._prev_ts_len:
                if ts:
                    xmin = ts[0]
                    xmax = ts[-1] if ts[-1] > xmin else xmin + 1.0
                    ax.set_xlim(xmin, xmax)
                else:
                    ax.set_xlim(0, max(len(data), self.window))
            # Autoscale Y less frequently to reduce work
            if (self._frame % self._autoscale_every) == 0:
                try:
                    ax.relim()
                    ax.autoscale_view(scalex=False, scaley=True)
                except Exception:
                    pass

        # Compass
        azi = self._get_float(last.get("tilt_az_deg360"))
        tilt = self._get_float(last.get("tilt_deg"))
        self.compass.update(azi, tilt)

        # Raw
        if self.raw_visible and (self._frame % self._raw_update_every) == 0:
            self._set_text(self.raw_text, "\n".join(reversed(raw)))

        self._frame += 1
        self._prev_ts_len = len(ts)

        return tuple(self.lines.values())

    @staticmethod
    def _get_float(v: object) -> Optional[float]:
        if v is None:
            return None
        try:
            f = float(v)
            return f if not math.isnan(f) else None
        except Exception:
            return None

    @staticmethod
    def _get_int(v: object) -> Optional[int]:
        if v is None:
            return None
        try:
            return int(float(v))
        except Exception:
            return None

    def run(self) -> None:
        # Interval derived from FPS for smoother yet efficient updates
        fps = getattr(getattr(self, 'args', object()), 'fps', 20)
        interval_ms = int(1000 / max(1, int(fps)))
        self.ani = animation.FuncAnimation(self.fig, self._update, interval=interval_ms, blit=False)
        plt.show()

    def _set_text(self, artist, s: str) -> None:
        key = id(artist)
        if self._text_cache.get(key) != s:
            artist.set_text(s)
            self._text_cache[key] = s

    def _reload_serial(self) -> None:
        # Increment generation so current reader exits
        try:
            self._reader_id += 1
        except Exception:
            self._reader_id = 1
        # Close current port
        try:
            self.ser.close()
        except Exception:
            pass
        # Determine port/baud
        try:
            port = getattr(self.ser, 'port', None) or getattr(self, '_port', None)
            baud = getattr(self.ser, 'baudrate', None) or getattr(self, '_baud', 115200)
        except Exception:
            port = getattr(self, '_port', None)
            baud = getattr(self, '_baud', 115200)
        if not port:
            return
        # Reopen
        try:
            self.ser = serial.Serial(port, baudrate=int(baud), timeout=1)
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass
            # Start a fresh reader thread for this generation
            self._reader = threading.Thread(target=self._reader_loop, args=(self._reader_id,), name="serial-reader", daemon=True)
            self._reader.start()
            self._port, self._baud = port, int(baud)
        except Exception as e:
            try:
                import sys
                print(f"[flight_visualizer] Reload failed for {port}: {e}", file=sys.stderr)
            except Exception:
                pass


# ---------------------------------------------------------------------------
# Entrypoint


def main() -> None:
    args = parse_args()
    with serial.Serial(args.port, baudrate=args.baud, timeout=1) as ser:
        try:
            ser.reset_input_buffer()
        except Exception:
            pass
        viz = FlightVisualizer(ser, args.window, show_raw=args.show_raw, raw_buffer=args.raw_buffer, print_raw=args.print_raw)
        # pass fps to instance for interval calculation
        viz.args = args  # type: ignore
        viz.run()


if __name__ == "__main__":
    main()
