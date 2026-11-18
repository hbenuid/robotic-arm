"""Launcher: find the CANable's serial port and run the motor-control CLI on it.

    uv run launcher                    # auto-detect + run
    uv run launcher --bitrate 250000   # extra arguments go to motor-control

With --channel, an --interface other than slcan (socketcan names a netdev, not a port) or --fake, nothing is detected:
the arguments go to motor-control as they are.
"""

from __future__ import annotations

import sys

from serial.tools import list_ports

from robotic_arm import motor_control


def find_can_port() -> str:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports detected. Plug the CANable in and try again.")
        sys.exit(1)

    # Several ports: prefer the ones whose description / manufacturer mentions CANable.
    if len(ports) > 1:
        likely = [
            p for p in ports
            if "canable" in (p.description or "").lower() or "canable" in (p.manufacturer or "").lower()
        ]
        if likely:
            ports = likely
        if len(ports) > 1:
            shown = ", ".join(f"{p.device} ({p.description or '?'})" for p in ports)
            print(f"Multiple candidates: {shown}. Using first; pass --channel to override.")
    return ports[0].device


def main() -> None:
    extra = sys.argv[1:]
    args = motor_control.parse_args(extra)
    channel_given = any(a == "--channel" or a.startswith("--channel=") for a in extra)
    if channel_given or args.interface != "slcan" or args.fake:
        motor_control.main(extra)
        return
    port = find_can_port()
    print(f"Auto-detected CAN port: {port}")
    motor_control.main(["--channel", port, *extra])


if __name__ == "__main__":
    main()
