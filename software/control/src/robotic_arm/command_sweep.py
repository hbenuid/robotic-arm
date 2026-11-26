"""One board's command check (the club's scripts/test_all_commands.py), made hard to run by accident.

    uv run python -m robotic_arm.command_sweep --interface slcan --channel /dev/ttyACM0 --can-id 1
    uv run python -m robotic_arm.command_sweep --interface slcan --channel /dev/ttyACM0 --can-id 1 --motion

Without --motion it only reads (nothing moves): 0x40, 0x38, 0x31, 0x30, 0x32, 0xF1, 0x3A, 0x39, 0x34, 0x3E, 0x3B.
--motion adds, after a typed confirmation at a terminal: an enable / disable cycle, a move of --move-motor-deg (degrees
of the MOTOR, default 90; the joint turns that through its reduction) out and back, a short slow spin, an emergency stop,
and a disable. A failed motion check stops the motor and ends the run.

It talks to the one CAN id given on the command line - never to the config's table, whose ids are unconfirmed -, needs
the interface and channel named outright, and has no console script. Ctrl+C sends that board an emergency stop.
"""

from __future__ import annotations

import argparse
import signal
import sys
import time
from collections.abc import Iterator
from dataclasses import dataclass

from robotic_arm import config
from robotic_arm.can_interface import CANInterface
from robotic_arm.motor_driver import STATUS_NAMES, Cmd, MotorDriver

MAX_MOVE_MOTOR_DEG = 360
MAX_SPEED_RPM = 300
POSITION_TOLERANCE = 200   # counts, ~4.4 deg of the motor


@dataclass
class Check:
    name: str
    passed: bool | None   # None: skipped
    detail: str

    def __str__(self) -> str:
        tag = {None: "\033[33m[SKIP]\033[0m", True: "\033[32m[PASS]\033[0m", False: "\033[31m[FAIL]\033[0m"}[self.passed]
        return f"  {tag} {self.name:40s}: {self.detail}"


def check_read(motor: MotorDriver, code: int, name: str, min_len: int) -> Check:
    """A read command answers with its code and at least `min_len` bytes (code included, check byte not)."""
    reply = motor.request(bytes([code]))
    if reply is None:
        return Check(name, False, "No response")
    if len(reply.data) < min_len:
        return Check(name, False, f"Short response: {reply.data.hex(' ')}")
    return Check(name, True, f"OK ({reply.data[1:].hex(' ')})")


READS = [
    (Cmd.READ_VERSION, "Read version (0x40)", 4),
    (Cmd.READ_POWER, "Read voltage + current (0x38, V1.0.9)", 5),
    (Cmd.READ_ENCODER, "Read encoder cumulative (0x31)", 7),
    (Cmd.READ_ENCODER_CARRY, "Read encoder carry (0x30)", 7),
    (Cmd.READ_SPEED, "Read speed (0x32)", 3),
    (Cmd.READ_STATUS, "Read motor status (0xF1)", 2),
    (Cmd.READ_ENABLED, "Read enable status (0x3A)", 2),
    (Cmd.READ_ANGLE_ERROR, "Read shaft angle error (0x39)", 5),
    (Cmd.READ_IO, "Read IO port status (0x34)", 2),
    (Cmd.READ_STALL, "Read protection status (0x3E)", 2),
    (Cmd.READ_HOME_STATUS, "Read home status (0x3B)", 2),
]


def read_checks(motor: MotorDriver) -> Iterator[Check]:
    for code, name, min_len in READS:
        yield check_read(motor, code, name, min_len)


def check_enable(motor: MotorDriver, enable: bool) -> Check:
    name = f"{'Enable' if enable else 'Disable'} motor (0xF3)"
    if not motor.enable(enable):
        return Check(name, False, "No response or refused")
    actual = motor.read_enabled()
    ok = actual is enable
    return Check(name, ok, f"0x3A reads {'no response' if actual is None else ('enabled' if actual else 'disabled')}")


def check_position(motor: MotorDriver, expected: int) -> Check:
    name = f"Encoder ~{expected} (0x31)"
    value = motor.read_encoder()
    if value is None:
        return Check(name, False, "No response")
    return Check(name, abs(value - expected) <= POSITION_TOLERANCE, f"{value} counts (delta {value - expected:+d})")


def travel_s(counts: int, rpm: int) -> float:
    """The move's time at full speed, plus ramps and slack."""
    return abs(counts) / config.ENCODER_COUNTS_PER_REV / (rpm / 60.0) + 3.0


def motion_checks(motor: MotorDriver, move_counts: int, rpm: int) -> Iterator[Check]:
    """Each check in turn; the caller stops at the first failure."""
    yield check_enable(motor, True)
    yield check_enable(motor, False)
    yield check_enable(motor, True)
    time.sleep(0.5)   # the board settles after enabling

    start = motor.read_encoder()
    yield Check("Read start position (0x31)", start is not None, "No response" if start is None else f"{start} counts")
    if start is None:   # the caller stopped at that failure; nothing below runs without a start
        return
    target = start + move_counts
    yield Check(f"Absolute move to {target} (0xF5)", motor.move_to_axis(target, rpm, 2), "accepted or not")
    stopped = motor.wait_stopped(travel_s(move_counts, rpm))
    yield Check("Stopped after the move (0xF1)", stopped, STATUS_NAMES[1] if stopped else "not stopped in time")
    yield check_position(motor, target)
    yield Check(f"Relative move {-move_counts:+d} (0xF4)", motor.move_relative_axis(-move_counts, rpm, 2), "accepted or not")
    stopped = motor.wait_stopped(travel_s(move_counts, rpm))
    yield Check("Stopped after the move (0xF1)", stopped, STATUS_NAMES[1] if stopped else "not stopped in time")
    yield check_position(motor, start)

    spin_s = min(0.5, abs(move_counts) / config.ENCODER_COUNTS_PER_REV / (rpm / 60.0))
    yield Check(f"Speed mode {rpm} RPM for {spin_s:.2f} s (0xF6)", motor.set_speed(0, rpm, 2), "running or not")
    time.sleep(spin_s)
    yield Check("Emergency stop (0xF7)", motor.emergency_stop(), "confirmed or not")
    time.sleep(1.0)
    speed = motor.read_speed()
    yield Check("Speed ~0 (0x32)", speed is not None and abs(speed) <= 20, "No response" if speed is None else f"{speed} RPM")
    yield check_enable(motor, False)


def what_it_moves(can_id: int, move_motor_deg: float) -> str:
    """The move at each joint of config.JOINTS - the CAN id's joint is unconfirmed."""
    lines = []
    for jc in config.JOINTS:
        mark = "  <- config's joint for this id (unconfirmed)" if jc.can_id == can_id else ""
        lines.append(f"    {jc.name:15s} {move_motor_deg * jc.gear_ratio:6.1f} deg{mark}")
    return "\n".join(lines)


def confirm_motion(args: argparse.Namespace) -> bool:
    if not sys.stdin.isatty():
        print("--motion needs a terminal to confirm at; refusing.")
        return False
    print(f"\n  WARNING: board {args.can_id:#04x} will be enabled, DISABLED (a loaded joint falls - support the arm),\n"
          f"  moved {args.move_motor_deg} deg of its motor out and back at {args.speed} RPM, spun briefly, stopped and\n"
          f"  disabled. Put its joint mid-range first. At each joint that is:\n{what_it_moves(args.can_id, args.move_motor_deg)}")
    return input("  Type 'MOVE' to go on: ").strip() == "MOVE"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="python -m robotic_arm.command_sweep", description=__doc__.split("\n\n")[0])
    parser.add_argument("--interface", required=True, help="python-can interface: slcan, socketcan, virtual ...")
    parser.add_argument("--channel", required=True, help="the adapter's port or netdev")
    parser.add_argument("--can-id", required=True, type=lambda s: int(s, 0), help="the one board to check (1-2047)")
    parser.add_argument("--bitrate", type=int, default=config.CAN_BITRATE, help="(not used by socketcan / virtual)")
    parser.add_argument("--motion", action="store_true", help="also enable, move, spin, stop and disable the motor")
    parser.add_argument("--move-motor-deg", type=float, default=90.0, help=f"the move, motor degrees (1-{MAX_MOVE_MOTOR_DEG})")
    parser.add_argument("--speed", type=int, default=30, help=f"motor RPM for the moves (1-{MAX_SPEED_RPM})")
    args = parser.parse_args(argv)
    if not 1 <= args.can_id <= 0x7FF:
        parser.error("--can-id must be 1-2047")
    if not 1 <= args.move_motor_deg <= MAX_MOVE_MOTOR_DEG:
        parser.error(f"--move-motor-deg must be 1-{MAX_MOVE_MOTOR_DEG}")
    if not 1 <= args.speed <= MAX_SPEED_RPM:
        parser.error(f"--speed must be 1-{MAX_SPEED_RPM}")
    return args


def run(motor: MotorDriver, checks: Iterator[Check], stop_on_failure: bool) -> list[Check]:
    results = []
    for check in checks:
        results.append(check)
        print(check)
        if stop_on_failure and check.passed is False:
            print("  A motion check failed: emergency stop, the rest skipped.")
            motor.emergency_stop()
            break
    return results


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.motion and not confirm_motion(args):
        print("Not confirmed; nothing sent.")
        return 1
    bitrate = None if args.interface in ("socketcan", "virtual") else args.bitrate
    try:
        can = CANInterface(args.channel, args.interface, bitrate)
    except Exception as e:  # noqa: BLE001 - serial, python-can or OS: any failure to open ends the run
        print(f"Failed to open CAN bus: {e}")
        return 1
    motor = MotorDriver(can, args.can_id)

    def on_sigint(_signum: int, _frame: object) -> None:
        print(f"\n!!! EMERGENCY STOP {args.can_id:#04x} !!!")
        motor.emergency_stop()
        sys.exit(1)

    previous = signal.signal(signal.SIGINT, on_sigint)
    print(f"Board {args.can_id:#04x} on {args.channel} ({args.interface})\n")
    try:
        results = run(motor, read_checks(motor), stop_on_failure=False)
        if args.motion:
            move_counts = round(args.move_motor_deg / 360.0 * config.ENCODER_COUNTS_PER_REV)
            print()
            results += run(motor, motion_checks(motor, move_counts, args.speed), stop_on_failure=True)
    finally:
        signal.signal(signal.SIGINT, previous)
        can.close()
    passed = sum(c.passed is True for c in results)
    print(f"\n  {passed}/{len(results)} passed")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
