#!/usr/bin/env python3
"""Very simple serial monitor for quick testing.

Usage examples:

    python tools/serial_monitor.py --list
    python tools/serial_monitor.py --port COM3
    python tools/serial_monitor.py --port /dev/tty.usbmodemXXXX --baud 115200

Requires: pyserial (pip install pyserial)
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Optional

import serial
from serial.tools import list_ports


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Print incoming serial lines to stdout")
    p.add_argument("--port", help="Serial port (e.g. COM3 or /dev/tty.usbmodemXXXX)")
    p.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    p.add_argument("--list", action="store_true", help="List available serial ports and exit")
    p.add_argument(
        "--timestamp",
        action="store_true",
        help="Prefix each line with a HH:MM:SS timestamp",
    )
    return p.parse_args()


def list_available_ports() -> None:
    ports = list_ports.comports()
    if not ports:
        print("No serial ports found.")
        return
    print("Available serial ports:\n------------------------")
    for p in ports:
        desc = f" ({p.description})" if p.description else ""
        hwid = f" [{p.hwid}]" if p.hwid else ""
        print(f"- {p.device}{desc}{hwid}")


def open_serial(port: str, baud: int) -> serial.Serial:
    try:
        ser = serial.Serial(port, baudrate=baud, timeout=1)
    except serial.SerialException as e:
        print(f"Error opening {port} @ {baud} baud: {e}", file=sys.stderr)
        sys.exit(2)
    return ser


def format_line(s: str, *, timestamp: bool) -> str:
    if timestamp:
        ts = time.strftime("%H:%M:%S")
        return f"[{ts}] {s}"
    return s


def run_monitor(port: str, baud: int, *, timestamp: bool) -> int:
    print(f"Opening {port} @ {baud} baud. Press Ctrl+C to exit.")
    with open_serial(port, baud) as ser:
        try:
            while True:
                raw = ser.readline()
                if not raw:
                    continue
                try:
                    line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                except Exception:
                    # Fallback if decoding goes really wrong
                    line = repr(raw)
                print(format_line(line, timestamp=timestamp))
        except KeyboardInterrupt:
            print("\nExiting.")
            return 0
        except serial.SerialException as e:
            print(f"\nSerial error: {e}", file=sys.stderr)
            return 1


def main() -> None:
    args = parse_args()

    if args.list:
        list_available_ports()
        return

    if not args.port:
        print("--port is required unless using --list", file=sys.stderr)
        sys.exit(2)

    code = run_monitor(args.port, args.baud, timestamp=args.timestamp)
    sys.exit(code)


if __name__ == "__main__":
    main()

