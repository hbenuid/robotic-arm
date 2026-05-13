#!/usr/bin/env python3
"""Launcher: auto-detect the CAN adapter port and run motor_control.py.

Usage:
    uv run launcher                    # auto-detect + run
    uv run launcher --bitrate 250000   # extra args forward to motor_control
"""

from __future__ import annotations

import sys

from serial.tools import list_ports


def find_can_port() -> str:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports detected. Plug the CANable in and try again.")
        sys.exit(1)

    # When multiple ports exist, prefer ones whose description/manufacturer
    # mentions CANable. Otherwise fall back to all USB devices.
    if len(ports) > 1:
        likely = [
            p for p in ports
            if "canable" in (p.description or "").lower()
            or "canable" in (p.manufacturer or "").lower()
        ]
        if likely:
            ports = likely
        if len(ports) > 1:
            shown = ", ".join(f"{p.device} ({p.description or '?'})" for p in ports)
            print(f"Multiple candidates: {shown}. Using first; pass --channel to override.")

    return ports[0].device


def main() -> None:
    port = find_can_port()
    print(f"Auto-detected CAN port: {port}")

    extra = sys.argv[1:]
    sys.argv = ["motor_control.py", "--channel", port, *extra]

    import motor_control
    motor_control.main()


if __name__ == "__main__":
    main()
