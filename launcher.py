#!/usr/bin/env python3
"""Launcher: auto-detect the CAN adapter port and run motor_control.py.

Usage:
    uv run launcher                    # auto-detect + run
    uv run launcher --bitrate 250000   # extra args forward to motor_control
"""

from __future__ import annotations

import glob
import sys

CANABLE_GLOB = "/dev/tty.usbmodem*"  # macOS path for CANable / slcan adapters


def find_can_port() -> str:
    ports = sorted(glob.glob(CANABLE_GLOB))
    if not ports:
        print(f"No CAN adapter found at {CANABLE_GLOB}.")
        print("Plug the CANable in and try again.")
        sys.exit(1)
    if len(ports) > 1:
        print(f"Multiple adapters: {ports}. Using first; pass --channel to override.")
    return ports[0]


def main() -> None:
    port = find_can_port()
    print(f"Auto-detected CAN port: {port}")

    extra = sys.argv[1:]
    sys.argv = ["motor_control.py", "--channel", port, *extra]

    import motor_control
    motor_control.main()


if __name__ == "__main__":
    main()
