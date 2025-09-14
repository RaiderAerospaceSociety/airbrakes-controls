#!/usr/bin/env python3
"""Compatibility wrapper: this tool has been renamed to flight_visualizer.py.

Usage remains similar:
    python tools/flight_visualizer.py --port COM3  # or /dev/tty.usbmodemXXXX
"""

import sys


def main() -> None:
    print("[tilt_visualizer] Renamed to tools/flight_visualizer.py — forwarding...", file=sys.stderr)
    # Re-exec by importing and calling the new main
    try:
        from tools.flight_visualizer import main as flight_main  # type: ignore
    except Exception:
        # Fallback import when run as a script from tools/
        try:
            from flight_visualizer import main as flight_main  # type: ignore
        except Exception as e:
            print("Failed to import flight_visualizer: ", e, file=sys.stderr)
            sys.exit(1)
    # Forward original argv (drop our script name)
    flight_main()


if __name__ == "__main__":
    main()
