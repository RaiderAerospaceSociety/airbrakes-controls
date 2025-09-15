#!/usr/bin/env python3
"""Qt Flight Visualizer: PySide6 + PyQtGraph version

Replicates one of each current structures from the Matplotlib visualizer:
- Battery and temperature gauges (horizontal bar style)
- State text, lockout indicator, clocks, and error counters
- Status lights panels (composable via nested VBox/HBox layouts)
- Airbrake dual-needle dial (cmd/act degrees)
- Compass (azimuth needle + tilt text)
- Timeseries plots for agl_fused_m, vz_fused_mps, az_imu1_mps2
- Collapsible raw serial monitor

Dependencies:
    pip install PySide6 pyqtgraph

Usage:
    python tools/flight_visualizer_qt.py --port COM3 --baud 115200

Notes:
- Uses QtSerialPort for event-driven reads (no threads required).
- Layout is built from nested QVBoxLayout/QHBoxLayout to mimic SwiftUI stacks.
- PyQtGraph is used for fast timeseries plotting.
"""

from __future__ import annotations

import argparse
import math
from collections import deque
import time
from typing import Deque, Dict, List, Optional
import sys

from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtSerialPort import QSerialPort, QSerialPortInfo

import numpy as np
import pyqtgraph as pg


# ------------------------------ CLI ----------------------------------------


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Qt Flight Visualizer")
    p.add_argument("--port", required=True, help="Serial port, e.g. COM3 or /dev/tty.usbmodem123")
    p.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    p.add_argument("--window", type=int, default=300, help="Timeseries window (samples)")
    p.add_argument("--fps", type=int, default=20, help="UI update rate (frames per second)")
    p.add_argument("--plot-fps", type=int, default=15, help="Max plot update rate (frames per second)")
    p.add_argument("--show-raw", action="store_true", help="Start with raw monitor expanded")
    p.add_argument("--raw-buffer", type=int, default=400, help="Raw lines kept in monitor")
    p.add_argument("--print-raw", action="store_true", help="Also print raw lines to stdout")
    p.add_argument("--screen-idx", type=int, help="Target screen index (see --list-screens)")
    p.add_argument("--screen-name", type=str, help="Substring match for target screen name")
    p.add_argument("--list-screens", action="store_true", help="List available screens and exit")
    p.add_argument("--fast", action="store_true", help="Favor performance (no AA, try OpenGL)")
    p.add_argument("--high-quality", action="store_true", help="High quality rendering (AA on, no OpenGL)")
    p.add_argument("--light", action="store_true", help="Light theme palette")
    p.add_argument("--components", action="store_true", help="Show fused components for altitude and velocity")
    return p.parse_args()


# ---------------------------- Small helpers --------------------------------


def clamp(v: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(vmax, v))


def fmt_time(s: Optional[float]) -> str:
    if s is None or math.isnan(s):
        return "--:--.--"
    s = max(0.0, float(s))
    m = int(s // 60)
    rem = s - m * 60
    return f"{m:02d}:{rem:05.2f}"


# ------------------------------- Widgets -----------------------------------


class HBarGaugeWidget(QtWidgets.QWidget):
    """Simple horizontal bar gauge using QProgressBar and a label."""

    def __init__(self, title: str, vmin: float, vmax: float, unit: str = "", parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self.vmin = float(vmin)
        self.vmax = float(vmax)
        self.unit = unit
        self._value: Optional[float] = None

        self.title_label = QtWidgets.QLabel(title)
        self.title_label.setStyleSheet("font-weight: 600;")
        self.value_label = QtWidgets.QLabel("--")
        self.value_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.bar = QtWidgets.QProgressBar()
        self.bar.setRange(int(self.vmin * 1000), int(self.vmax * 1000))
        self.bar.setTextVisible(False)

        top = QtWidgets.QHBoxLayout()
        top.addWidget(self.title_label)
        top.addStretch(1)
        top.addWidget(self.value_label)
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addLayout(top)
        lay.addWidget(self.bar)

    def update_value(self, value: Optional[float]) -> None:
        if value is None or math.isnan(value):
            self.value_label.setText("--")
            self.bar.setValue(int(self.vmin * 1000))
            return
        v = clamp(float(value), self.vmin, self.vmax)
        self.value_label.setText(f"{v:.3f}{self.unit}")
        self.bar.setValue(int(v * 1000))


class Light(QtWidgets.QWidget):
    def __init__(self, diameter: int = 16, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self._state: Optional[bool] = None
        self._diam = diameter
        self.setFixedSize(diameter, diameter)

    def set_state(self, st: Optional[bool]) -> None:
        self._state = st
        self.update()

    def paintEvent(self, e: QtGui.QPaintEvent) -> None:
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        r = QtCore.QRectF(1, 1, self._diam - 2, self._diam - 2)
        if self._state is None:
            color = QtGui.QColor("#555555")
        else:
            color = QtGui.QColor("#2ca02c" if self._state else "#d62728")
        p.setPen(QtGui.QPen(QtGui.QColor("#aaaaaa"), 1))
        p.setBrush(QtGui.QBrush(color))
        p.drawEllipse(r)


class LightWithLabel(QtWidgets.QWidget):
    def __init__(self, name: str, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self.name = name
        self.light = Light()
        self.label = QtWidgets.QLabel(name)
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)
        lay.addWidget(self.light)
        lay.addWidget(self.label)
        lay.addStretch(1)

    def set_state(self, st: Optional[bool]) -> None:
        self.light.set_state(st)


class LightsPanelWidget(QtWidgets.QWidget):
    """A panel that arranges a list of lights vertically or horizontally.
    You can compose several of these with HBox/VBox around them to get
    arbitrary stack layouts.
    """

    def __init__(self, names: List[str], orientation: QtCore.Qt.Orientation = QtCore.Qt.Horizontal, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self.names = names
        self._items: Dict[str, LightWithLabel] = {}
        if orientation == QtCore.Qt.Horizontal:
            lay = QtWidgets.QHBoxLayout(self)
        else:
            lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(12)
        for name in self.names:
            item = LightWithLabel(name)
            lay.addWidget(item)
            self._items[name] = item
        lay.addStretch(1)

    def update_states(self, states: Dict[str, Optional[bool]]) -> None:
        for name, item in self._items.items():
            item.set_state(states.get(name))


class DialGaugeDualWidget(QtWidgets.QWidget):
    """Semicircular dial with two needles (cmd/act)."""

    def __init__(self, vmin: float, vmax: float, label: str = "Airbrake deg", parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self.vmin = float(vmin)
        self.vmax = float(vmax)
        self.label = label
        self.cmd: Optional[float] = None
        self.act: Optional[float] = None
        self.setMinimumSize(180, 140)

    def set_values(self, cmd: Optional[float], act: Optional[float]) -> None:
        self.cmd, self.act = cmd, act
        self.update()

    def _val_to_angle_deg(self, v: float) -> float:
        v = clamp(v, self.vmin, self.vmax)
        frac = (v - self.vmin) / (self.vmax - self.vmin) if self.vmax > self.vmin else 0.0
        return -120.0 + frac * 240.0

    def paintEvent(self, e: QtGui.QPaintEvent) -> None:
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        rect = self.rect()
        cx, cy = rect.width() / 2.0, rect.height() * 0.65
        radius = min(rect.width() * 0.45, rect.height() * 0.55)

        # Arc
        p.setPen(QtGui.QPen(QtGui.QColor("#aaaaaa"), 2))
        # draw arc from -120 to +120 degrees
        start_angle = int((-120) * 16)
        span_angle = int((240) * 16)
        p.drawArc(QtCore.QRectF(cx - radius, cy - radius, 2 * radius, 2 * radius), start_angle, span_angle)

        # Ticks and labels
        p.setPen(QtGui.QPen(QtGui.QColor("#999999"), 2))
        n_major = 6
        for i in range(n_major + 1):
            v = self.vmin + i * (self.vmax - self.vmin) / n_major
            ang = math.radians(self._val_to_angle_deg(v))
            x0, y0 = cx + math.cos(ang) * radius * 0.88, cy + math.sin(ang) * radius * 0.88
            x1, y1 = cx + math.cos(ang) * radius, cy + math.sin(ang) * radius
            p.drawLine(int(x0), int(y0), int(x1), int(y1))
            tx, ty = cx + math.cos(ang) * radius * 0.72, cy + math.sin(ang) * radius * 0.72
            p.setPen(QtGui.QPen(QtGui.QColor("#cccccc")))
            p.drawText(QtCore.QRectF(tx - 14, ty - 8, 28, 16), QtCore.Qt.AlignCenter, f"{v:.0f}")

        # Label
        p.setPen(QtGui.QPen(QtGui.QColor("#dddddd")))
        p.drawText(QtCore.QRectF(0, cy + 8, rect.width(), 20), QtCore.Qt.AlignHCenter | QtCore.Qt.AlignTop, self.label)

        # Needles
        def draw_needle(value: Optional[float], color: str):
            if value is None or math.isnan(value):
                return
            ang = math.radians(self._val_to_angle_deg(float(value)))
            x1, y1 = cx + math.cos(ang) * radius * 0.98, cy + math.sin(ang) * radius * 0.98
            p.setPen(QtGui.QPen(QtGui.QColor(color), 3))
            p.drawLine(int(cx), int(cy), int(x1), int(y1))

        draw_needle(self.cmd, "#1f77b4")
        draw_needle(self.act, "#d62728")

        # Value text
        txts = []
        if self.cmd is not None and not math.isnan(self.cmd):
            txts.append(f"cmd {self.cmd:.1f}°")
        if self.act is not None and not math.isnan(self.act):
            txts.append(f"act {self.act:.1f}°")
        p.drawText(QtCore.QRectF(0, cy - 20, rect.width(), 20), QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter, "  |  ".join(txts))


class CompassWidget(QtWidgets.QWidget):
    def __init__(self, label: str = "Azimuth / Tilt", parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self.azi: Optional[float] = None
        self.tilt: Optional[float] = None
        self.label = label
        self.setMinimumSize(180, 140)

    def set_values(self, azi_deg360: Optional[float], tilt_deg: Optional[float]) -> None:
        self.azi = azi_deg360
        self.tilt = tilt_deg
        self.update()

    @staticmethod
    def _azi_to_math_rad(azi_deg: float) -> float:
        # 0° = North; map to math 0° = +x and CCW
        return math.radians(90.0 - azi_deg)

    def paintEvent(self, e: QtGui.QPaintEvent) -> None:
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        rect = self.rect()
        cx, cy = rect.width() / 2.0, rect.height() / 2.0
        radius = min(rect.width(), rect.height()) * 0.40

        # Circle
        p.setPen(QtGui.QPen(QtGui.QColor("#aaaaaa"), 2))
        p.drawEllipse(QtCore.QPointF(cx, cy), radius, radius)

        # Cardinal labels (E, N, W, S)
        p.setPen(QtGui.QPen(QtGui.QColor("#cccccc")))
        for ang_deg, lab in ((0, "E"), (90, "N"), (180, "W"), (270, "S")):
            ang = math.radians(ang_deg)
            x, y = cx + math.cos(ang) * radius * 1.08, cy + math.sin(ang) * radius * 1.08
            p.drawText(QtCore.QRectF(x - 10, y - 8, 20, 16), QtCore.Qt.AlignCenter, lab)

        # Tilt vector with length mapping and color map
        if self.azi is not None and not math.isnan(self.azi):
            ang = self._azi_to_math_rad(float(self.azi))
            # length scale: clamp at 90
            tilt_val = 0.0 if (self.tilt is None or math.isnan(self.tilt)) else float(self.tilt)
            clamped = max(0.0, min(tilt_val, 90.0))
            length = (clamped / 90.0) * radius
            # color: 0-30 green, 30-60 yellow, 60-90 red, >90 magenta
            if tilt_val <= 30:
                color = QtGui.QColor(44, 160, 44)  # green
            elif tilt_val <= 60:
                color = QtGui.QColor(255, 191, 0)  # yellow
            elif tilt_val <= 90:
                color = QtGui.QColor(214, 39, 40)  # red
            else:
                color = QtGui.QColor(198, 120, 221)  # magenta for >90
            x1, y1 = cx + math.cos(ang) * length, cy + math.sin(ang) * length
            p.setPen(QtGui.QPen(color, 3))
            p.drawLine(int(cx), int(cy), int(x1), int(y1))

        # Center text
        p.setPen(QtGui.QPen(QtGui.QColor("#dddddd")))
        tilt_txt = "--" if (self.tilt is None or math.isnan(self.tilt)) else f"{self.tilt:.2f}°"
        p.drawText(QtCore.QRectF(0, cy - 10, rect.width(), 20), QtCore.Qt.AlignCenter, f"tilt: {tilt_txt}")
        p.drawText(QtCore.QRectF(0, rect.height() - 22, rect.width(), 20), QtCore.Qt.AlignCenter, self.label)


class TiltPolarWidget(QtWidgets.QWidget):
    def __init__(self, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self.azi: Optional[float] = None
        self.tilt: Optional[float] = None
        self.setMinimumSize(180, 140)

    def set_values(self, azi_deg360: Optional[float], tilt_deg: Optional[float]) -> None:
        self.azi = azi_deg360
        self.tilt = tilt_deg
        self.update()

    def paintEvent(self, e: QtGui.QPaintEvent) -> None:
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        rect = self.rect()
        cx, cy = rect.width() / 2.0, rect.height() / 2.0
        radius = min(rect.width(), rect.height()) * 0.40
        # Outer circle and 90° ring
        p.setPen(QtGui.QPen(QtGui.QColor('#aaaaaa'), 2))
        p.drawEllipse(QtCore.QPointF(cx, cy), radius, radius)
        p.setPen(QtGui.QPen(QtGui.QColor('#666666'), 1, QtCore.Qt.DashLine))
        p.drawEllipse(QtCore.QPointF(cx, cy), radius, radius)
        # Vector
        if self.azi is not None and self.tilt is not None and not math.isnan(self.azi) and not math.isnan(self.tilt):
            ang = math.radians(90.0 - float(self.azi))
            tilt_val = float(self.tilt)
            clamped = max(0.0, min(tilt_val, 90.0))
            length = (clamped / 90.0) * radius
            if tilt_val <= 30:
                color = QtGui.QColor(44, 160, 44)
            elif tilt_val <= 60:
                color = QtGui.QColor(255, 191, 0)
            elif tilt_val <= 90:
                color = QtGui.QColor(214, 39, 40)
            else:
                color = QtGui.QColor(198, 120, 221)
            x1, y1 = cx + math.cos(ang) * length, cy + math.sin(ang) * length
            p.setPen(QtGui.QPen(color, 3))
            p.drawLine(int(cx), int(cy), int(x1), int(y1))
        # Label
        p.setPen(QtGui.QPen(QtGui.QColor('#dddddd')))
        p.drawText(QtCore.QRectF(0, rect.height() - 18, rect.width(), 16), QtCore.Qt.AlignCenter, 'Tilt Polar')


class Tilt3DWidget(QtWidgets.QWidget):
    def __init__(self, parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        try:
            import pyqtgraph.opengl as gl
            self._have_gl = True
            self._gl = gl
            self.view = gl.GLViewWidget()
            lay = QtWidgets.QVBoxLayout(self)
            lay.setContentsMargins(0, 0, 0, 0)
            lay.addWidget(self.view)
            self.view.opts['distance'] = 4
            # Axes
            ax = gl.GLAxisItem(size=QtGui.QVector3D(1,1,1))
            self.view.addItem(ax)
            # Rocket direction line
            self._arrow = gl.GLLinePlotItem(pos=np.array([[0,0,0],[0,0,1]], dtype=np.float32), color=(1,0,0,1), width=2)
            self.view.addItem(self._arrow)
        except Exception:
            self._have_gl = False
            lab = QtWidgets.QLabel('3D view unavailable')
            lab.setAlignment(QtCore.Qt.AlignCenter)
            lay = QtWidgets.QVBoxLayout(self)
            lay.setContentsMargins(0, 0, 0, 0)
            lay.addWidget(lab)
        self.azi: Optional[float] = None
        self.tilt: Optional[float] = None

    def set_values(self, azi_deg360: Optional[float], tilt_deg: Optional[float]) -> None:
        self.azi = azi_deg360
        self.tilt = tilt_deg
        if not self._have_gl or self.azi is None or self.tilt is None:
            return
        # Rocket axis vector based on tilt from +Z and azimuth
        theta = math.radians(float(self.tilt))
        phi = math.radians(float(self.azi))
        # Clamp tilt to [0, pi]
        theta = max(0.0, min(theta, math.pi))
        vx = math.sin(theta) * math.cos(phi)
        vy = math.sin(theta) * math.sin(phi)
        vz = math.cos(theta)
        pos = np.array([[0,0,0],[vx, vy, vz]], dtype=np.float32)
        self._arrow.setData(pos=pos)


class TextBlock(QtWidgets.QWidget):
    """Convenience widget with labeled lines of text updated programmatically."""

    def __init__(self, lines: List[str], parent: Optional[QtWidgets.QWidget] = None):
        super().__init__(parent)
        self.labels: Dict[str, QtWidgets.QLabel] = {}
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        for name in lines:
            lbl = QtWidgets.QLabel(f"{name}: --")
            self.labels[name] = lbl
            lay.addWidget(lbl)
        lay.addStretch(1)

    def set_value(self, name: str, text: str) -> None:
        if name in self.labels:
            self.labels[name].setText(f"{name}: {text}")


# ----------------------------- Main Window ---------------------------------


class FlightVisualizerQt(QtWidgets.QMainWindow):
    def __init__(self, port: str, baud: int, window: int, fps: int, show_raw: bool, raw_buffer: int, print_raw: bool, fast: bool = True, plot_fps: int = 15, components: bool = False, light_theme: bool = False):
        super().__init__()
        self.setWindowTitle("Flight Visualizer (Qt)")
        self.resize(1600, 900)

        self.print_raw = bool(print_raw)
        self.fast = bool(fast)
        self.components = bool(components)
        self.light_theme = bool(light_theme)
        self._updating = False
        self._last_plot_t = 0.0
        self._plot_interval = 1.0 / max(1, int(plot_fps))
        self._last_abs_ts_ms: Optional[float] = None

        # Data buffers/state
        self.window = int(window)
        self.ts: Deque[float] = deque(maxlen=self.window)
        self._t0_ms: Optional[float] = None
        self._dt_guess = 0.05
        self.last: Dict[str, object] = {}
        self.raw_lines: Deque[str] = deque(maxlen=int(raw_buffer))
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
            "lockout",
        ]
        self.flags: Dict[str, Optional[bool]] = {k: None for k in self.flag_names}
        self.ts_metrics = [
            ("agl_fused_m", None, None),
            ("vz_fused_mps", None, None),
            ("az_imu1_mps2", None, None),
        ]
        self.metric_data: Dict[str, Deque[float]] = {k: deque(maxlen=self.window) for k, _, _ in self.ts_metrics}
        # Target x-span (seconds) once buffer reaches full window; then scroll
        self._x_span_s: Optional[float] = None
        # Track Y range updates to avoid frequent autoscale thrash
        self._y_last_update: Dict[str, int] = {}
        # Previous flags for edge-detect
        self._prev_flags: Dict[str, Optional[bool]] = {}
        # Event markers per plot
        self._event_markers: Dict[str, List[tuple]] = {"agl_fused_m": [], "vz_fused_mps": [], "az_imu1_mps2": []}
        self._marker_limits = {"liftoff_det": 2, "burnout_det": 2, "tilt_latch": 2, "baro_agree": 1000}
        self._event_codes = {"liftoff_det": "LIF", "burnout_det": "BO", "tilt_latch": "TLT", "baro_agree": "BAR"}
        # Component series buffers (for overlays)
        self.comp_metrics = ["agl_bmp1_m", "agl_imu1_m", "vz_mps", "vz_baro_mps", "vz_acc_mps"]
        self.comp_data: Dict[str, Deque[float]] = {k: deque(maxlen=self.window) for k in self.comp_metrics}

        # ---------------- Layout (stacks) ----------------
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # Top area as a single row with 6 columns:
        # [Dial over Dial] | [Light Indicators] | [HGauge over HGauge] | [State+Lockout] | [Timers] | [Errors]
        top_row = QtWidgets.QHBoxLayout()
        top_row.setSpacing(8)
        root.addLayout(top_row)

        # Column 1: Dial over Dial (Airbrake dial over Compass)
        dials_col_widget = QtWidgets.QWidget()
        dials_col = QtWidgets.QVBoxLayout(dials_col_widget)
        dials_col.setContentsMargins(0, 0, 0, 0)
        dials_col.setSpacing(6)
        self.brake_dial = DialGaugeDualWidget(0.0, 90.0, label="Airbrake deg")
        self.compass = CompassWidget(label="Azimuth / Tilt")
        dials_col.addWidget(self.brake_dial)
        dials_col.addWidget(self.compass)
        top_row.addWidget(dials_col_widget, 2)

        # Column 2: Light Indicators (two vertical stacks inside an HStack)
        lights_row_widget = QtWidgets.QWidget()
        lights_row = QtWidgets.QHBoxLayout(lights_row_widget)
        lights_row.setContentsMargins(0, 0, 0, 0)
        lights_row.setSpacing(8)
        self.lights_sensors = LightsPanelWidget(["sens_imu1_ok", "sens_bmp1_ok", "sens_imu2_ok"], orientation=QtCore.Qt.Vertical)
        self.lights_status = LightsPanelWidget(["tilt_ok", "mach_ok"], orientation=QtCore.Qt.Vertical)
        lights_row.addWidget(self.lights_sensors)
        lights_row.addWidget(self.lights_status)
        top_row.addWidget(lights_row_widget, 1)

        # Column 3: HGauge over HGauge (Battery over Temp)
        self.batt_gauge = HBarGaugeWidget("Battery (V)", 3.0, 4.2, unit="V")
        self.temp_gauge = HBarGaugeWidget("Temp (F)", 14.0, 176.0, unit="°F")
        gauges_col_widget = QtWidgets.QWidget()
        gauges_col = QtWidgets.QVBoxLayout(gauges_col_widget)
        gauges_col.setContentsMargins(0, 0, 0, 0)
        gauges_col.setSpacing(6)
        gauges_col.addWidget(self.batt_gauge)
        gauges_col.addWidget(self.temp_gauge)
        # Temp sparkline (15s)
        self.spark_temp = pg.PlotWidget()
        self._setup_spark(self.spark_temp)
        self.spark_temp.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.spark_temp.setFixedHeight(32)
        lbl_temp = QtWidgets.QLabel("Temp (°C) — last 15 s")
        lbl_temp.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        gauges_col.addWidget(lbl_temp)
        self._spark_temp_curve = self.spark_temp.plot([], [], pen=pg.mkPen('#ff7f0e', width=1))
        self._spark_temp_buf: Deque[tuple] = deque(maxlen=1800)
        gauges_col.addWidget(self.spark_temp)
        top_row.addWidget(gauges_col_widget, 1)

        # Column 4: State and Lockout
        self.state_block = TextBlock(["STATE", "LOCKOUT"])  # LOCKOUT shows ON/OFF
        top_row.addWidget(self.state_block, 1)

        # Column 5: Timers + tilt sparkline
        timers_col_widget = QtWidgets.QWidget()
        timers_col = QtWidgets.QVBoxLayout(timers_col_widget)
        timers_col.setContentsMargins(0, 0, 0, 0)
        timers_col.setSpacing(6)
        self.clock_block = TextBlock(["Alive", "Since liftoff", "To apogee"]) 
        timers_col.addWidget(self.clock_block)
        self.spark_tilt = pg.PlotWidget()
        self._setup_spark(self.spark_tilt)
        self.spark_tilt.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        self.spark_tilt.setFixedHeight(32)
        lbl_tilt = QtWidgets.QLabel("Tilt (deg) — last 15 s")
        lbl_tilt.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        timers_col.addWidget(lbl_tilt)
        self._spark_tilt_curve = self.spark_tilt.plot([], [], pen=pg.mkPen('#1f77b4', width=1))
        self._spark_tilt_buf: Deque[tuple] = deque(maxlen=1800)
        timers_col.addWidget(self.spark_tilt)
        top_row.addWidget(timers_col_widget, 1)

        # Column 6: Errors
        self.err_block = TextBlock(["i2c_errs", "spi_errs"]) 
        top_row.addWidget(self.err_block, 1)

        # (Polar/3D tilt views removed; keeping only compass)

        # Timeseries row using pyqtgraph
        ts_row = QtWidgets.QHBoxLayout()
        ts_row.setSpacing(8)
        root.addLayout(ts_row, stretch=1)

        pg.setConfigOptions(
            antialias=not self.fast,
            background="#ffffff" if self.light_theme else "#12161c",
            foreground="#222222" if self.light_theme else "#e6e6e6",
        )
        # Try enabling OpenGL if --fast is set (avoid on macOS due to driver issues)
        try:
            if self.fast and not sys.platform.startswith('darwin'):
                pg.setConfigOptions(useOpenGL=True)
        except Exception:
            pass
        self.ts_plots: Dict[str, pg.PlotWidget] = {}
        self.ts_curves: Dict[str, pg.PlotDataItem] = {}
        display_names = {
            "agl_fused_m": "Altitude (m)",
            "vz_fused_mps": "Vertical Velocity (m/s)",
            "az_imu1_mps2": "Vertical Acceleration (m/s²)",
        }
        # component overlay curve refs
        self.alt_comp_curves: Dict[str, pg.PlotDataItem] = {}
        self.vel_comp_curves: Dict[str, pg.PlotDataItem] = {}
        for name, ymin, ymax in self.ts_metrics:
            pw = pg.PlotWidget()
            pw.showGrid(x=True, y=True, alpha=0.3)
            pw.setTitle(display_names.get(name, name))
            pw.setLabel("left", display_names.get(name, name))
            # We'll manage Y range manually with hysteresis; disable auto-Y
            pw.enableAutoRange(x=False, y=False)
            if ymin is not None and ymax is not None:
                pw.setYRange(ymin, ymax)
            curve = pw.plot([], [], pen=pg.mkPen(width=2))
            ts_row.addWidget(pw, 1)
            self.ts_plots[name] = pw
            self.ts_curves[name] = curve
            if name == "agl_fused_m":
                leg = pw.addLegend()
                try:
                    leg.anchor((1, 0), (1, 0))
                except Exception:
                    pass
                self.ts_curves[name].setPen(pg.mkPen('#00d1ff', width=2))
                try:
                    leg.addItem(self.ts_curves[name], "fused")
                except Exception:
                    pass
                # Component curves
                self.alt_comp_curves["agl_bmp1_m"] = pw.plot([], [], pen=pg.mkPen((255,127,14,180), width=2))
                self.alt_comp_curves["agl_imu1_m"] = pw.plot([], [], pen=pg.mkPen((148,103,189,180), width=2))
                if self.components:
                    try:
                        leg.addItem(self.alt_comp_curves["agl_bmp1_m"], "baro")
                        leg.addItem(self.alt_comp_curves["agl_imu1_m"], "imu1")
                    except Exception:
                        pass
                else:
                    self.alt_comp_curves["agl_bmp1_m"].hide()
                    self.alt_comp_curves["agl_imu1_m"].hide()
            elif name == "vz_fused_mps":
                leg = pw.addLegend()
                try:
                    leg.anchor((1, 0), (1, 0))
                except Exception:
                    pass
                self.ts_curves[name].setPen(pg.mkPen('#2ca02c', width=2))
                try:
                    leg.addItem(self.ts_curves[name], "fused")
                except Exception:
                    pass
                self.vel_comp_curves["vz_mps"] = pw.plot([], [], pen=pg.mkPen((31,119,180,180), width=2))
                self.vel_comp_curves["vz_acc_mps"] = pw.plot([], [], pen=pg.mkPen((214,39,40,180), width=2))
                if self.components:
                    try:
                        leg.addItem(self.vel_comp_curves["vz_mps"], "baro/deriv")
                        leg.addItem(self.vel_comp_curves["vz_acc_mps"], "accel")
                    except Exception:
                        pass
                else:
                    self.vel_comp_curves["vz_mps"].hide()
                    self.vel_comp_curves["vz_acc_mps"].hide()

        # Raw monitor (collapsible)
        self.raw_visible = bool(show_raw)
        self.raw_box = QtWidgets.QPlainTextEdit()
        self.raw_box.setReadOnly(True)
        self.raw_box.setMaximumBlockCount(int(raw_buffer))
        self.raw_box.setVisible(self.raw_visible)
        self.raw_box.setStyleSheet("font-family: Menlo, Consolas, monospace; font-size: 11px;")
        root.addWidget(self.raw_box, stretch=1)

        # Status bar hints
        self.statusBar().showMessage("Press P to toggle Raw, R to reload serial")

        # Serial
        self._port_name = port
        self._baud = baud
        self.ser = QSerialPort(self)
        self.ser.setPortName(self._port_name)
        self.ser.setBaudRate(self._baud)
        self.ser.setReadBufferSize(4096)
        self.ser.readyRead.connect(self._on_ready_read)
        try:
            self.ser.errorOccurred.connect(self._on_serial_error)
        except Exception:
            pass
        self._rx_buf = bytearray()
        if not self.ser.open(QtCore.QIODevice.ReadOnly):
            QtWidgets.QMessageBox.critical(self, "Serial", f"Failed to open {self._port_name}")

        # Update timer
        self._frame = 0
        self._prev_ts_len = 0
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(int(1000 / max(1, int(fps))))
        self.timer.timeout.connect(self._update_ui)
        self.timer.start()

        # Throttle raw text updates to reduce UI overhead
        self._raw_pending: List[str] = []
        self._raw_flush_timer = QtCore.QTimer(self)
        self._raw_flush_timer.setInterval(200)  # ms
        self._raw_flush_timer.timeout.connect(self._flush_raw_box)
        self._raw_flush_timer.start()

    # --------------------------- Input handling ---------------------------
    def keyPressEvent(self, e: QtGui.QKeyEvent) -> None:
        k = e.key()
        if k in (QtCore.Qt.Key_P, ):
            self.raw_visible = not self.raw_visible
            self.raw_box.setVisible(self.raw_visible)
        elif k in (QtCore.Qt.Key_R, ):
            # Send soft reset command to MCU, then clear locally
            try:
                if self.ser.isOpen():
                    self.ser.write(b"!cmd:soft_reset\n")
                    self.ser.flush()
            except Exception:
                pass
            self._reload_serial(clear=True)
        elif k in (QtCore.Qt.Key_C, ):
            # Clear all buffers/plots without touching serial
            self._clear_data()
        else:
            super().keyPressEvent(e)

    def closeEvent(self, e: QtGui.QCloseEvent) -> None:
        try:
            if self.ser.isOpen():
                self.ser.close()
        except Exception:
            pass
        super().closeEvent(e)

    # --------------------------- Serial parsing ---------------------------
    def _on_ready_read(self) -> None:
        try:
            self._rx_buf.extend(self.ser.readAll().data())
        except Exception:
            return
        while True:
            nl = self._rx_buf.find(b"\n")
            if nl < 0:
                break
            raw = self._rx_buf[:nl].rstrip(b"\r")
            del self._rx_buf[: nl + 1]
            try:
                line = raw.decode("utf-8", errors="replace")
            except Exception:
                line = repr(raw)
            if self.print_raw:
                print(line)
            self._handle_line(line)

    def _handle_line(self, line: str) -> None:
        # Parse key:value CSV; accept teleplot leading '>'
        parts = [p.strip() for p in line.split(',') if p.strip()]
        ts_ms: Optional[float] = None
        values: Dict[str, object] = {}
        for ch in parts:
            if ':' not in ch:
                continue
            k, vs = ch.split(':', 1)
            k = k.strip()
            if k.startswith('>'):
                k = k[1:].strip()
            vs = vs.strip()
            # Out-of-band events from device
            if k == "evt":
                vsl = vs.lower()
                if vsl in ("soft_reset", "hard_reset"):
                    self._clear_data()
                # ignore event pairs in data parsing
                continue
            if k in ("ts_ms", "ts"):
                try:
                    ts_ms = float(vs)
                except ValueError:
                    ts_ms = None
                continue
            if k == "fc_state_str":
                values[k] = vs
                continue
            try:
                values[k] = float(vs)
            except ValueError:
                values[k] = vs

        # Only treat as telemetry if timestamp is present (ignore log-only lines)
        is_telem = (ts_ms is not None)

        # Detect timestamp reset (device reboot) and clear state
        if is_telem:
            if self._last_abs_ts_ms is not None and ts_ms + 1000.0 < self._last_abs_ts_ms:
                self._clear_data()
            self._last_abs_ts_ms = ts_ms

        # Update buffers/state only for telemetry lines
        if is_telem:
            if self._t0_ms is None:
                self._t0_ms = ts_ms
            t = max(0.0, (ts_ms - self._t0_ms) / 1000.0)
            self.ts.append(t)

            # Capture latest scalar values
            self.last.update(values)
            st = str(self.last.get("fc_state_str", ""))
            if "lockout" not in values:
                self.flags["lockout"] = True if st.upper() == "ABORT_LOCKOUT" else False if st else None
            for name in self.flag_names:
                if name in values:
                    try:
                        self.flags[name] = (float(values[name]) != 0.0)
                    except Exception:
                        pass
            # Timeseries metrics (append only when the line is telemetry)
            for name in self.metric_data.keys():
                v = values.get(name)
                try:
                    vf = float(v) if v is not None else float('nan')
                except Exception:
                    vf = float('nan')
                self.metric_data[name].append(vf)
            # Components timeseries (for overlays)
            for cname in getattr(self, 'comp_metrics', []):
                v = values.get(cname)
                try:
                    vf = float(v) if v is not None else float('nan')
                except Exception:
                    vf = float('nan')
                self.comp_data[cname].append(vf)

        self.raw_lines.append(line)
        if self.raw_visible:
            self._raw_pending.append(line)
            # Prevent unbounded growth if serial is too fast
            if len(self._raw_pending) > 200:
                self._flush_raw_box()

    # ----------------------------- UI update ------------------------------
    def _get_float(self, v: object) -> Optional[float]:
        if v is None:
            return None
        try:
            f = float(v)
            return f if not math.isnan(f) else None
        except Exception:
            return None

    def _get_int(self, v: object) -> Optional[int]:
        if v is None:
            return None
        try:
            return int(float(v))
        except Exception:
            return None

    def _setup_spark(self, pw: pg.PlotWidget) -> None:
        pw.setMenuEnabled(False)
        pw.hideAxis('left')
        pw.hideAxis('bottom')
        pw.showGrid(x=False, y=False)
        pw.setMouseEnabled(x=False, y=False)
        # Make it look like a mini version of the full plots
        bg = "#ffffff" if getattr(self, 'light_theme', False) else "#12161c"
        pw.setBackground(bg)
        # Add a subtle border to delineate the spark area
        border = "#cccccc" if getattr(self, 'light_theme', False) else "#444444"
        try:
            pw.setStyleSheet(f"border: 1px solid {border}; border-radius: 4px;")
        except Exception:
            pass
        try:
            vb = pw.getPlotItem().getViewBox()
            vb.setPadding(0.02)
        except Exception:
            pass

    def _update_ui(self) -> None:
        if self._updating:
            return
        self._updating = True
        ts = list(self.ts)
        last = dict(self.last)
        flags = dict(self.flags)

        # Gauges
        self.batt_gauge.update_value(self._get_float(last.get("vbat_v")))
        tc = self._get_float(last.get("temp_c"))
        tf = (tc * 9.0 / 5.0 + 32.0) if tc is not None else None
        self.temp_gauge.update_value(tf)

        # State + lockout
        st = str(last.get("fc_state_str", "")) if last.get("fc_state_str") is not None else ""
        self.state_block.set_value("STATE", st if st else "--")
        lock = flags.get("lockout")
        lock_txt = "ON" if lock else ("OFF" if lock is not None else "--")
        self.state_block.set_value("LOCKOUT", lock_txt)

        # Clocks
        t_alive = ts[-1] if ts else None
        t_since = self._get_float(last.get("t_since_launch_s"))
        t_to_ap = self._get_float(last.get("t_to_apogee_s"))
        self.clock_block.set_value("Alive", fmt_time(t_alive))
        self.clock_block.set_value("Since liftoff", fmt_time(t_since))
        self.clock_block.set_value("To apogee", fmt_time(t_to_ap))

        # Errors
        i2c_errs = self._get_int(last.get("i2c_errs"))
        spi_errs = self._get_int(last.get("spi_errs"))
        self.err_block.set_value("i2c_errs", str(i2c_errs) if i2c_errs is not None else "--")
        self.err_block.set_value("spi_errs", str(spi_errs) if spi_errs is not None else "--")

        # Lights
        self.lights_sensors.update_states(flags)
        self.lights_status.update_states(flags)

        # Airbrake dial
        cmd = self._get_float(last.get("cmd_deg"))
        act = self._get_float(last.get("act_deg"))
        self.brake_dial.set_values(cmd, act)

        # Compass + tilt views
        azi = self._get_float(last.get("tilt_az_deg360"))
        tilt = self._get_float(last.get("tilt_deg"))
        self.compass.set_values(azi, tilt)
        # Polar/3D tilt views removed for simplicity

        # Sparklines (15s window by time)
        tnow = ts[-1] if ts else 0.0
        if tilt is not None:
            self._spark_tilt_buf.append((tnow, tilt))
        if tc is not None:
            self._spark_temp_buf.append((tnow, tc))
        # prune and plot
        def update_spark(buf: Deque[tuple], curve: pg.PlotDataItem, span_s: float = 15.0):
            if not buf:
                return
            tmax = buf[-1][0]
            # drop old
            while buf and (tmax - buf[0][0]) > span_s:
                buf.popleft()
            xs = np.array([p[0] for p in buf], dtype=np.float32)
            ys = np.array([p[1] for p in buf], dtype=np.float32)
            # shift x to start at 0 for stable ranges
            xs = xs - xs[0]
            curve.setData(xs, ys, clipToView=True, downsampleMethod='peak', autoDownsample=True)
            # keep a fixed-span x range [0, span_s]
            try:
                pw = curve.getViewBox().parent()
            except Exception:
                pw = None
            if hasattr(self, 'spark_temp') and (curve is self._spark_temp_curve):
                try:
                    self.spark_temp.setXRange(0, span_s, padding=0)
                except Exception:
                    pass
            if hasattr(self, 'spark_tilt') and (curve is self._spark_tilt_curve):
                try:
                    self.spark_tilt.setXRange(0, span_s, padding=0)
                except Exception:
                    pass
        # decide whether to do plotting work this tick
        now = time.monotonic()
        do_plots = (now - self._last_plot_t) >= self._plot_interval
        if do_plots:
            update_spark(self._spark_tilt_buf, self._spark_tilt_curve)
            update_spark(self._spark_temp_buf, self._spark_temp_curve)

        # Timeseries
        # Establish scrolling span once window is full (when using timestamps)
        if ts and self._x_span_s is None and len(ts) >= self.window:
            span = float(ts[-1] - ts[0])
            if span > 0:
                self._x_span_s = span

        for name, data in self.metric_data.items():
            # Use numpy arrays and enable downsampling/clip-to-view to keep fast
            if do_plots:
                if ts and len(ts) == len(data):
                    xx = np.asarray(ts, dtype=np.float32)
                else:
                    xx = np.arange(len(data), dtype=np.float32)
                yy = np.asarray(list(data), dtype=np.float32)
                self.ts_curves[name].setData(xx, yy, autoDownsample=True, clipToView=True, downsampleMethod='peak')
                # Component overlays
                if self.components:
                    if name == "agl_fused_m" and self.alt_comp_curves:
                        b = np.asarray(list(self.comp_data.get("agl_bmp1_m", [])), dtype=np.float32)
                        i = np.asarray(list(self.comp_data.get("agl_imu1_m", [])), dtype=np.float32)
                        if b.size:
                            self.alt_comp_curves["agl_bmp1_m"].show()
                            nb = int(min(len(xx), len(b)))
                            if nb > 0:
                                self.alt_comp_curves["agl_bmp1_m"].setData(xx[-nb:], b[-nb:], autoDownsample=True, clipToView=True, downsampleMethod='peak')
                        if i.size:
                            self.alt_comp_curves["agl_imu1_m"].show()
                            ni = int(min(len(xx), len(i)))
                            if ni > 0:
                                self.alt_comp_curves["agl_imu1_m"].setData(xx[-ni:], i[-ni:], autoDownsample=True, clipToView=True, downsampleMethod='peak')
                    elif name == "vz_fused_mps" and self.vel_comp_curves:
                        v1 = np.asarray(list(self.comp_data.get("vz_mps", [])), dtype=np.float32)
                        if v1.size == 0:
                            v1 = np.asarray(list(self.comp_data.get("vz_baro_mps", [])), dtype=np.float32)
                        v2 = np.asarray(list(self.comp_data.get("vz_acc_mps", [])), dtype=np.float32)
                        if v1.size:
                            self.vel_comp_curves["vz_mps"].show()
                            n1 = int(min(len(xx), len(v1)))
                            if n1 > 0:
                                self.vel_comp_curves["vz_mps"].setData(xx[-n1:], v1[-n1:], autoDownsample=True, clipToView=True, downsampleMethod='peak')
                        if v2.size:
                            self.vel_comp_curves["vz_acc_mps"].show()
                            n2 = int(min(len(xx), len(v2)))
                            if n2 > 0:
                                self.vel_comp_curves["vz_acc_mps"].setData(xx[-n2:], v2[-n2:], autoDownsample=True, clipToView=True, downsampleMethod='peak')

            # Keep the view showing the active window and scroll after width reached
            pw = self.ts_plots.get(name)
            if pw is None:
                continue
            if ts and len(ts) == len(data):
                if self._x_span_s is None:
                    xmin = float(ts[0]) if ts else 0.0
                    xmax = float(ts[-1]) if ts else float(self.window)
                    if xmax <= xmin:
                        xmax = xmin + 1.0
                else:
                    xmax = float(ts[-1])
                    xmin = xmax - float(self._x_span_s)
                if do_plots:
                    pw.setXRange(xmin, xmax, padding=0)
                # Hysteretic manual Y autoscale to avoid thrash on noisy signals
                if do_plots:
                    try:
                        finite = np.isfinite(yy)
                        if finite.any():
                            mn = float(yy[finite].min())
                            mx = float(yy[finite].max())
                            # Include component ranges when enabled so they don't get clipped off-screen
                            if self.components:
                                if name == "agl_fused_m":
                                    b = np.asarray(list(self.comp_data.get("agl_bmp1_m", [])), dtype=np.float32)
                                    i = np.asarray(list(self.comp_data.get("agl_imu1_m", [])), dtype=np.float32)
                                    for arr in (b, i):
                                        if arr.size:
                                            msk = np.isfinite(arr)
                                            if msk.any():
                                                mn = min(mn, float(arr[msk].min()))
                                                mx = max(mx, float(arr[msk].max()))
                                elif name == "vz_fused_mps":
                                    v1 = np.asarray(list(self.comp_data.get("vz_mps", [])), dtype=np.float32)
                                    if v1.size == 0:
                                        v1 = np.asarray(list(self.comp_data.get("vz_baro_mps", [])), dtype=np.float32)
                                    v2 = np.asarray(list(self.comp_data.get("vz_acc_mps", [])), dtype=np.float32)
                                    for arr in (v1, v2):
                                        if arr.size:
                                            msk = np.isfinite(arr)
                                            if msk.any():
                                                mn = min(mn, float(arr[msk].min()))
                                                mx = max(mx, float(arr[msk].max()))
                            if not math.isfinite(mn) or not math.isfinite(mx):
                                raise ValueError
                            # Current view Y range
                            y0, y1 = pw.viewRange()[1]
                            span = max(1e-3, y1 - y0)
                            # Update if data exceeds by a margin or every 30 frames
                            margin = 0.10 * span
                            needs_expand = (mn < y0 + margin) or (mx > y1 - margin)
                            needs_periodic = (self._y_last_update.get(name, -1) < 0) or ((self._frame - self._y_last_update.get(name, 0)) >= 30)
                            if needs_expand or needs_periodic:
                                # Target range with padding based on data span
                                dspan = max(1e-3, mx - mn)
                                pad = max(0.1 * dspan, 0.01)
                                new_y0 = mn - pad
                                new_y1 = mx + pad
                                if new_y1 <= new_y0:
                                    new_y1 = new_y0 + 1.0
                                pw.setYRange(new_y0, new_y1, padding=0)
                                self._y_last_update[name] = self._frame
                    except Exception:
                        pass
            else:
                n = len(data)
                if n <= 1:
                    continue
                if n < self.window:
                    xmin, xmax = 0.0, float(n)
                else:
                    xmax = float(n)
                    xmin = float(n - self.window)
                if do_plots:
                    pw.setXRange(xmin, xmax, padding=0)

        # Event markers (edges)
        try:
            self._update_event_markers(ts, flags)
        except Exception:
            pass

        if do_plots:
            self._last_plot_t = now
        self._frame += 1
        self._prev_ts_len = len(ts)
        self._updating = False

    # --------------------------- Serial helpers ---------------------------
    def _on_serial_error(self, err):
        # Attempt reconnect on resource errors; leave others
        try:
            from PySide6.QtSerialPort import QSerialPort
            recoverable = err in (
                QSerialPort.ResourceError,
                QSerialPort.DeviceNotFoundError,
                QSerialPort.PermissionError,
            )
        except Exception:
            recoverable = True
        if recoverable:
            self._schedule_reconnect()

    # ------------------------------ Events ---------------------------------
    def _update_event_markers(self, ts: List[float], flags: Dict[str, Optional[bool]]) -> None:
        if not ts:
            return
        x = float(ts[-1])
        # Detect rising edges
        events = ["liftoff_det", "burnout_det", "tilt_latch", "baro_agree"]
        for ev in events:
            prev = self._prev_flags.get(ev)
            cur = flags.get(ev)
            if prev is not None and cur is not None and (not prev) and cur:
                self._add_marker(ev, x)
            self._prev_flags[ev] = cur
        # prune markers outside left bound for each plot
        for plot_name, markers in self._event_markers.items():
            pw = self.ts_plots.get(plot_name)
            if not pw:
                continue
            try:
                xmin, xmax = pw.viewRange()[0]
            except Exception:
                xmin = x - 1e9
            keep = []
            for line, text, ev in markers:
                try:
                    xval = float(line.value())
                except Exception:
                    try:
                        xval = float(line.pos().x())
                    except Exception:
                        xval = xmin
                if xval >= xmin - 1.0:
                    keep.append((line, text, ev))
                else:
                    try:
                        pw.removeItem(line)
                        pw.removeItem(text)
                    except Exception:
                        pass
            self._event_markers[plot_name] = keep

    def _add_marker(self, ev: str, x: float) -> None:
        code = self._event_codes.get(ev, ev[:3].upper())
        color_map = {
            "liftoff_det": (255, 255, 0, 160),
            "burnout_det": (255, 0, 0, 160),
            "tilt_latch": (255, 165, 0, 160),
            "baro_agree": (0, 200, 255, 120),
        }
        pen = pg.mkPen(color_map.get(ev, (200, 200, 200, 120)), width=1)
        # Assign plots per event
        targets = {
            "liftoff_det": ["agl_fused_m", "vz_fused_mps"],
            "burnout_det": ["vz_fused_mps", "az_imu1_mps2"],
            "tilt_latch": ["vz_fused_mps", "az_imu1_mps2"],
            "baro_agree": ["agl_fused_m"],
        }.get(ev, ["agl_fused_m"]) 
        for plot_name in targets:
            pw = self.ts_plots.get(plot_name)
            if not pw:
                continue
            line = pg.InfiniteLine(pos=x, angle=90, movable=False, pen=pen)
            pw.addItem(line)
            text = pg.TextItem(code, anchor=(0, 1))
            # place near top-right of current view
            try:
                (_, _), (ymin, ymax) = pw.viewRange()
                text.setPos(x, ymax)
            except Exception:
                text.setPos(x, 0.0)
            pw.addItem(text)
            # retention
            self._event_markers[plot_name].append((line, text, ev))
            limit = self._marker_limits.get(ev, 10)
            if len([m for m in self._event_markers[plot_name] if m[2] == ev]) > limit:
                # remove oldest of this type
                for idx, (l, t, e) in enumerate(self._event_markers[plot_name]):
                    if e == ev:
                        try:
                            pw.removeItem(l)
                            pw.removeItem(t)
                        except Exception:
                            pass
                        del self._event_markers[plot_name][idx]
                        break

    # ------------------------------ Serial ---------------------------------
    def _reload_serial(self, clear: bool = False) -> None:
        try:
            if self.ser.isOpen():
                self.ser.close()
        except Exception:
            pass
        try:
            if clear:
                self._clear_data()
            self.ser.setPortName(self._port_name)
            self.ser.setBaudRate(self._baud)
            self.ser.open(QtCore.QIODevice.ReadOnly)
        except Exception:
            QtWidgets.QMessageBox.warning(self, "Serial", "Reload failed")

    def _schedule_reconnect(self) -> None:
        try:
            if getattr(self, '_reconnect_timer', None) is None:
                self._reconnect_timer = QtCore.QTimer(self)
                self._reconnect_timer.setSingleShot(True)
                self._reconnect_timer.timeout.connect(lambda: self._reload_serial(clear=True))
            # try reconnect in ~1 second
            self._reconnect_timer.start(1000)
        except Exception:
            pass

    def _clear_data(self) -> None:
        # Clear timebase and data buffers
        self._t0_ms = None
        self._last_abs_ts_ms = None
        try:
            self.ts.clear()
        except Exception:
            self.ts = deque(maxlen=self.window)
        for d in self.metric_data.values():
            d.clear()
        for d in getattr(self, 'comp_data', {}).values():
            d.clear()
        self.last.clear()
        # Reset flags
        self.flags = {k: None for k in self.flag_names}
        self._y_last_update.clear()
        # Event markers removal
        try:
            for plot_name, markers in self._event_markers.items():
                pw = self.ts_plots.get(plot_name)
                if not pw:
                    continue
                for line, text, _ in markers:
                    try:
                        pw.removeItem(line)
                        pw.removeItem(text)
                    except Exception:
                        pass
                self._event_markers[plot_name] = []
        except Exception:
            pass
        # Clear curves
        try:
            for c in self.ts_curves.values():
                c.setData([], [])
            for c in getattr(self, 'alt_comp_curves', {}).values():
                c.setData([], [])
            for c in getattr(self, 'vel_comp_curves', {}).values():
                c.setData([], [])
        except Exception:
            pass
        # Reset X ranges to sane defaults
        try:
            for pw in self.ts_plots.values():
                pw.setXRange(0, max(1.0, float(self.window)), padding=0)
        except Exception:
            pass
        # Raw buffers
        try:
            self.raw_lines.clear()
            self._raw_pending.clear()
            if self.raw_visible:
                self.raw_box.clear()
        except Exception:
            pass
        # Sparklines
        try:
            self._spark_tilt_buf.clear()
            self._spark_temp_buf.clear()
            self._spark_tilt_curve.setData([], [])
            self._spark_temp_curve.setData([], [])
        except Exception:
            pass
        # Frame counters
        self._frame = 0
        self._prev_ts_len = 0

    def _flush_raw_box(self) -> None:
        if not self.raw_visible or not self._raw_pending:
            return
        try:
            chunk = "\n".join(self._raw_pending)
            self._raw_pending.clear()
            self.raw_box.appendPlainText(chunk)
        except Exception:
            self._raw_pending.clear()


def main() -> None:
    args = parse_args()

    # If requested, list screens and exit early
    if args.list_screens:
        # Create a minimal app to query screens
        app = QtWidgets.QApplication([])
        scrs = QtGui.QGuiApplication.screens()
        for i, s in enumerate(scrs):
            g = s.geometry()
            print(f"[{i}] {s.name()}  geom=({g.x()},{g.y()},{g.width()}x{g.height()})  primary={s is QtGui.QGuiApplication.primaryScreen()}")
        return

    app = QtWidgets.QApplication([])
    # Palette
    pal = app.palette()
    if getattr(args, 'light', False):
        pal.setColor(QtGui.QPalette.Window, QtGui.QColor("#f2f2f2"))
        pal.setColor(QtGui.QPalette.Base, QtGui.QColor("#ffffff"))
        pal.setColor(QtGui.QPalette.Text, QtGui.QColor("#222222"))
        pal.setColor(QtGui.QPalette.WindowText, QtGui.QColor("#222222"))
    else:
        pal.setColor(QtGui.QPalette.Window, QtGui.QColor("#0c0f14"))
        pal.setColor(QtGui.QPalette.Base, QtGui.QColor("#12161c"))
        pal.setColor(QtGui.QPalette.Text, QtGui.QColor("#e6e6e6"))
        pal.setColor(QtGui.QPalette.WindowText, QtGui.QColor("#e6e6e6"))
    app.setPalette(pal)

    # Fast by default; --high-quality disables, --fast forces on
    fast = True
    if getattr(args, 'high_quality', False):
        fast = False
    if getattr(args, 'fast', False):
        fast = True
    w = FlightVisualizerQt(
        args.port,
        args.baud,
        args.window,
        args.fps,
        args.show_raw,
        args.raw_buffer,
        args.print_raw,
        fast=fast,
        plot_fps=getattr(args, 'plot_fps', 15),
        components=getattr(args, 'components', False),
        light_theme=getattr(args, 'light', False),
    )
    w.show()

    # Place window on a specific screen, if requested
    try:
        target_screen = None
        screens = QtGui.QGuiApplication.screens()
        if getattr(args, 'screen_name', None):
            name_lc = args.screen_name.lower()
            for s in screens:
                if name_lc in (s.name() or "").lower():
                    target_screen = s
                    break
        if target_screen is None and getattr(args, 'screen_idx', None) is not None:
            idx = int(args.screen_idx)
            if 0 <= idx < len(screens):
                target_screen = screens[idx]
        if target_screen is not None:
            # Ensure window is associated with the target screen
            if w.windowHandle() is not None:
                try:
                    w.windowHandle().setScreen(target_screen)
                except Exception:
                    pass
            # Center within the target screen
            g = target_screen.geometry()
            # Avoid oversizing relative to screen
            new_w = min(w.width(), max(200, g.width() - 40))
            new_h = min(w.height(), max(200, g.height() - 80))
            w.resize(new_w, new_h)
            w.move(g.x() + (g.width() - new_w) // 2, g.y() + (g.height() - new_h) // 2)
    except Exception:
        pass
    app.exec()


if __name__ == "__main__":
    main()
