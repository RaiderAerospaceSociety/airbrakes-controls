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
from typing import Deque, Dict, List, Optional

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
    p.add_argument("--show-raw", action="store_true", help="Start with raw monitor expanded")
    p.add_argument("--raw-buffer", type=int, default=400, help="Raw lines kept in monitor")
    p.add_argument("--print-raw", action="store_true", help="Also print raw lines to stdout")
    p.add_argument("--screen-idx", type=int, help="Target screen index (see --list-screens)")
    p.add_argument("--screen-name", type=str, help="Substring match for target screen name")
    p.add_argument("--list-screens", action="store_true", help="List available screens and exit")
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

        # Heading needle
        if self.azi is not None and not math.isnan(self.azi):
            ang = self._azi_to_math_rad(float(self.azi))
            x1, y1 = cx + math.cos(ang) * radius, cy + math.sin(ang) * radius
            p.setPen(QtGui.QPen(QtGui.QColor("#1f77b4"), 3))
            p.drawLine(int(cx), int(cy), int(x1), int(y1))

        # Center text
        p.setPen(QtGui.QPen(QtGui.QColor("#dddddd")))
        tilt_txt = "--" if (self.tilt is None or math.isnan(self.tilt)) else f"{self.tilt:.2f}°"
        p.drawText(QtCore.QRectF(0, cy - 10, rect.width(), 20), QtCore.Qt.AlignCenter, f"tilt: {tilt_txt}")
        p.drawText(QtCore.QRectF(0, rect.height() - 22, rect.width(), 20), QtCore.Qt.AlignCenter, self.label)


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
    def __init__(self, port: str, baud: int, window: int, fps: int, show_raw: bool, raw_buffer: int, print_raw: bool):
        super().__init__()
        self.setWindowTitle("Flight Visualizer (Qt)")
        self.resize(1600, 900)

        self.print_raw = bool(print_raw)

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
        top_row.addWidget(gauges_col_widget, 1)

        # Column 4: State and Lockout
        self.state_block = TextBlock(["STATE", "LOCKOUT"])  # LOCKOUT shows ON/OFF
        top_row.addWidget(self.state_block, 1)

        # Column 5: Timers
        self.clock_block = TextBlock(["Alive", "Since liftoff", "To apogee"]) 
        top_row.addWidget(self.clock_block, 1)

        # Column 6: Errors
        self.err_block = TextBlock(["i2c_errs", "spi_errs"]) 
        top_row.addWidget(self.err_block, 1)

        # (Lights are in Column 2; dials stacked in Column 1)

        # Timeseries row using pyqtgraph
        ts_row = QtWidgets.QHBoxLayout()
        ts_row.setSpacing(8)
        root.addLayout(ts_row, stretch=1)

        pg.setConfigOptions(antialias=True, background="#12161c", foreground="#e6e6e6")
        self.ts_plots: Dict[str, pg.PlotWidget] = {}
        self.ts_curves: Dict[str, pg.PlotDataItem] = {}
        for name, ymin, ymax in self.ts_metrics:
            pw = pg.PlotWidget()
            pw.showGrid(x=True, y=True, alpha=0.3)
            pw.setTitle(name)
            pw.setLabel("left", name)
            pw.enableAutoRange(x=False, y=True)
            if ymin is not None and ymax is not None:
                pw.setYRange(ymin, ymax)
            curve = pw.plot([], [], pen=pg.mkPen(width=2))
            ts_row.addWidget(pw, 1)
            self.ts_plots[name] = pw
            self.ts_curves[name] = curve

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
        self.ser = QSerialPort(self)
        self.ser.setPortName(port)
        self.ser.setBaudRate(baud)
        self.ser.setReadBufferSize(4096)
        self.ser.readyRead.connect(self._on_ready_read)
        self._rx_buf = bytearray()
        if not self.ser.open(QtCore.QIODevice.ReadOnly):
            QtWidgets.QMessageBox.critical(self, "Serial", f"Failed to open {port}")

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
            self._reload_serial()
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

        # Update buffers/state
        if ts_ms is not None:
            if self._t0_ms is None:
                self._t0_ms = ts_ms
            t = max(0.0, (ts_ms - self._t0_ms) / 1000.0)
        else:
            t = (self.ts[-1] + self._dt_guess) if self.ts else 0.0
        self.ts.append(t)

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
        for name in self.metric_data.keys():
            v = values.get(name)
            try:
                vf = float(v) if v is not None else float('nan')
            except Exception:
                vf = float('nan')
            self.metric_data[name].append(vf)

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

    def _update_ui(self) -> None:
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

        # Compass
        azi = self._get_float(last.get("tilt_az_deg360"))
        tilt = self._get_float(last.get("tilt_deg"))
        self.compass.set_values(azi, tilt)

        # Timeseries
        for name, data in self.metric_data.items():
            # Use numpy arrays and enable downsampling/clip-to-view to keep fast
            if ts and len(ts) == len(data):
                xx = np.asarray(ts, dtype=float)
            else:
                xx = np.arange(len(data), dtype=float)
            yy = np.asarray(list(data), dtype=float)
            self.ts_curves[name].setData(xx, yy, autoDownsample=True, clipToView=True)

        self._frame += 1
        self._prev_ts_len = len(ts)

    # ------------------------------ Serial ---------------------------------
    def _reload_serial(self) -> None:
        try:
            if self.ser.isOpen():
                self.ser.close()
        except Exception:
            pass
        try:
            self.ser.open(QtCore.QIODevice.ReadOnly)
        except Exception:
            QtWidgets.QMessageBox.warning(self, "Serial", "Reload failed")

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
    # Dark-ish palette to match matplotlib version
    pal = app.palette()
    pal.setColor(QtGui.QPalette.Window, QtGui.QColor("#0c0f14"))
    pal.setColor(QtGui.QPalette.Base, QtGui.QColor("#12161c"))
    pal.setColor(QtGui.QPalette.Text, QtGui.QColor("#e6e6e6"))
    pal.setColor(QtGui.QPalette.WindowText, QtGui.QColor("#e6e6e6"))
    app.setPalette(pal)

    w = FlightVisualizerQt(args.port, args.baud, args.window, args.fps, args.show_raw, args.raw_buffer, args.print_raw)
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
