"""Interactive CLI for the arm's MKS boards: pick boards, read them, move them, configure them.

    uv run launcher                                              # finds a CANable (slcan) and starts this
    uv run motor-control --interface slcan --channel /dev/ttyACM0
    uv run motor-control --interface socketcan --channel can0    # candleLight firmware; bitrate set by `ip link`
    uv run motor-control --fake                                  # simulated boards on a virtual bus, no hardware

Ctrl+C sends an emergency stop to every board in config.JOINTS before exiting. Moves take encoder counts (the board's
own coordinate, 16384 per motor turn); the CLI shows what that is at the joint through its gear ratio.
"""

from __future__ import annotations

import argparse
import signal
import sys
from collections.abc import Callable
from dataclasses import dataclass

from robotic_arm import config
from robotic_arm.arm import Arm
from robotic_arm.can_interface import CANInterface
from robotic_arm.joint import Joint
from robotic_arm.motor_driver import (
    HARDWARE_NAMES,
    HOME_STATUS_NAMES,
    MAX_SPEED_RPM,
    STATUS_NAMES,
    Checksum,
    Cmd,
    MotorDriver,
    encode_uint16,
)

WORK_MODES = "0=CR_OPEN, 1=CR_CLOSE, 2=CR_vFOC, 3=SR_OPEN, 4=SR_CLOSE, 5=SR_vFOC"


@dataclass
class Target:
    """A selected board: its driver, and its joint when it is one of config.JOINTS."""

    label: str
    motor: MotorDriver
    joint: Joint | None


class Session:
    def __init__(self, arm: Arm, priority_estop: bool):
        self.arm = arm
        self.priority_estop = priority_estop
        self.selected: list[Target] = []

    @property
    def label(self) -> str:
        return ", ".join(t.label for t in self.selected) or "None"

    def emergency_stop_all(self) -> None:
        print("\n!!! EMERGENCY STOP ALL MOTORS !!!")
        confirmed = self.arm.emergency_stop(self.priority_estop)
        silent = [n for n, ok in confirmed.items() if not ok]
        print(f"Emergency stop sent to {len(confirmed)} boards" + (f"; no confirmation from {', '.join(silent)}." if silent else "."))

    def run(self, func: Callable[..., str], *args: object) -> None:
        if not self.selected:
            print("  No motor selected. Use option [0] first.")
            return
        for target in self.selected:
            print(f"  [{target.label}] {func(target, *args)}")

    def confirm_all(self) -> bool:
        return len(self.selected) <= 1 or confirm_dangerous("This will act on ALL selected motors.")


# --- input ------------------------------------------------------------------------------------------------------------


def confirm_dangerous(action: str) -> bool:
    return input(f"  WARNING: {action}\n  Type 'YES' to confirm: ").strip() == "YES"


def warn_high_speed(speed: int) -> bool:
    if speed > 1000:
        print(f"  [!] Speed {speed} RPM is above 1000: an immediate stop at this speed can damage the hardware.")
        return confirm_dangerous(f"Proceed with speed={speed} RPM?")
    return True


def input_int(prompt: str, default: int | None = None) -> int | None:
    suffix = f" [{default}]" if default is not None else ""
    raw = input(f"  {prompt}{suffix}: ").strip()
    if raw == "":
        return default
    try:
        return int(raw, 0)
    except ValueError:
        print("  Invalid number.")
        return None


def at_joint(target: Target, counts: int) -> str:
    if target.joint is None:
        return ""
    return f"  (= {target.joint.config.sign * counts / target.joint.counts_per_deg:+.2f} deg of {target.joint.name})"


def status_text(status: int | None, ok: tuple[int, ...] = (1,), what: str = "OK") -> str:
    if status is None:
        return "No response"
    return what if status in ok else f"Response: {status}"


# --- reads ------------------------------------------------------------------------------------------------------------


def cmd_read_encoder(t: Target) -> str:
    value = t.motor.read_encoder()
    return "No response" if value is None else f"Encoder: {value} counts from the board's zero{at_joint(t, value)}"


def cmd_read_encoder_carry(t: Target) -> str:
    value = t.motor.read_encoder_carry()
    return "No response" if value is None else f"Carry: {value[0]}, Value: {value[1]}"


def cmd_read_speed(t: Target) -> str:
    rpm = t.motor.read_speed()
    if rpm is None:
        return "No response"
    joint = f"  (= {t.joint.rpm_to_dps(rpm):.1f} deg/s at the joint)" if t.joint else ""
    return f"Speed: {rpm} RPM (positive CCW){joint}"


def cmd_read_status(t: Target) -> str:
    code = t.motor.read_status()
    return "No response" if code is None else f"Status: {STATUS_NAMES.get(code, f'Unknown({code})')}"


def cmd_read_enabled(t: Target) -> str:
    enabled = t.motor.read_enabled()
    return "No response" if enabled is None else ("Enabled" if enabled else "Disabled")


def cmd_read_angle_error(t: Target) -> str:
    error = t.motor.read_angle_error()
    return "No response" if error is None else f"Shaft error: {error} ({error * 360.0 / 51200.0:.2f} deg of the motor)"


def cmd_read_io(t: Target) -> str:
    s = t.motor.read_io()
    if s is None:
        return "No response"
    bits = (("IN1", 1), ("IN2", 2), ("OUT1", 4), ("OUT2", 8))
    return "IO: " + ", ".join(f"{name}={'HIGH' if s & bit else 'LOW'}" for name, bit in bits)


def cmd_read_version(t: Target) -> str:
    v = t.motor.read_version()
    if v is None:
        return "No response"
    return (f"Firmware {v.firmware_text} (bytes {v.firmware.hex(' ')}), board {HARDWARE_NAMES.get(v.hardware, v.hardware)}, "
            f"{'calibrated' if v.calibrated else 'NOT calibrated'}, bootloader {v.bootloader:#04x}")


def cmd_read_power(t: Target) -> str:
    p = t.motor.read_power()
    return "No response (0x38 is V1.0.9's)" if p is None else f"Supply {p.millivolts / 1000:.2f} V, motor {p.milliamps} mA"


def cmd_read_stall(t: Target) -> str:
    stalled = t.motor.read_stalled()
    return "No response" if stalled is None else ("PROTECTED (stalled)" if stalled else "Not protected")


def cmd_read_home_status(t: Target) -> str:
    statuses = t.motor.read_home_status()
    if statuses is None:
        return "No response"
    names = [HOME_STATUS_NAMES.get(s, f"Unknown({s})") for s in statuses]
    if len(names) == 2:
        return f"Single-turn zero: {names[0]}, homing: {names[1]}"
    return "Home status: " + ", ".join(names)


# --- motion -----------------------------------------------------------------------------------------------------------


def cmd_enable(t: Target, enable: bool) -> str:
    if not t.motor.enable(enable):
        return "No response or refused"
    return "OK - Enabled" if enable else "OK - Disabled"


def cmd_absolute_move(t: Target, speed: int, acc: int, position: int) -> str:
    return ("Move accepted" if t.motor.move_to_axis(position, speed, acc) else "No response or refused") + at_joint(t, position)


def cmd_relative_move(t: Target, speed: int, acc: int, offset: int) -> str:
    return ("Move accepted" if t.motor.move_relative_axis(offset, speed, acc) else "No response or refused") + at_joint(t, offset)


def cmd_speed_mode(t: Target, direction: int, speed: int, acc: int) -> str:
    return "Speed mode set" if t.motor.set_speed(direction, speed, acc) else "No response or refused"


# --- homing -----------------------------------------------------------------------------------------------------------


def cmd_go_home(t: Target, mode: int | None) -> str:
    status = t.motor.go_home(mode)
    meaning = {0: "failed", 1: "started", 2: "complete"}
    return "No response" if status is None else f"Homing: {meaning.get(status, f'status {status}')} (V1.0.9's meanings)"


def cmd_set_zero(t: Target) -> str:
    return "Zero set" if t.motor.set_zero() else "No response or refused"


def cmd_set_home_params(t: Target, trig: int, hdir: int, hspeed: int, elimit: int, mode: int | None) -> str:
    return "Home params set" if t.motor.set_home_params(trig, hdir, hspeed, bool(elimit), mode) else "No response or refused"


# --- configuration ----------------------------------------------------------------------------------------------------


def cmd_raw_status(t: Target, data: bytes, ok: tuple[int, ...] = (1,)) -> str:
    reply = t.motor.request(data)
    return status_text(None if reply is None else reply.status, ok)


def cmd_calibrate(t: Target) -> str:
    status = t.motor.calibrate()
    return {None: "No response", 0: "Still calibrating after 30 s", 1: "Calibration OK"}.get(status, f"Calibration failed ({status})")


def cmd_set_heartbeat(t: Target, ms: int) -> str:
    return "Heartbeat set" if t.motor.set_heartbeat(ms) else "No response or refused"


def cmd_set_checksum(t: Target, method: int) -> str:
    return f"Checksum now {Checksum(method).name}" if t.motor.set_checksum_method(Checksum(method)) else "No response or refused"


def cmd_restart(t: Target) -> str:
    reply = t.motor.request(bytes([Cmd.RESTART]))
    return "Restart sent (no reply)" if reply is None else status_text(reply.status, what="Restart acknowledged")


def cmd_emergency_stop(t: Target, priority: bool) -> str:
    return "Stopped" if t.motor.emergency_stop(priority) else "Stop sent (no confirmation)"


# --- menus ------------------------------------------------------------------------------------------------------------


def menu_select(s: Session) -> None:
    print("\n  --- Select Motor ---")
    joints = list(s.arm.joints.values())
    for i, j in enumerate(joints, 1):
        can_id = "no board" if j.motor is None else f"{j.motor.can_id:#04x}"
        print(f"  [{i}] {j.name:15s} ({can_id})")
    print("  [a] All mapped motors")
    print("  [i] A board by CAN id (any id, e.g. one not set yet)")
    print("  [b] Back")
    choice = input("  > ").strip().lower()
    if choice == "b":
        return
    if choice == "a":
        s.selected = [Target(j.name, j.board, j) for j in s.arm.mapped]
    elif choice == "i":
        can_id = input_int("CAN id (1-2047, 0x.. for hex)")
        if can_id is None or not 1 <= can_id <= 0x7FF:
            print("  Must be 1-2047.")
            return
        joint = next((j for j in s.arm.mapped if j.board.can_id == can_id), None)
        s.selected = [Target(f"{can_id:#04x}", joint.board if joint else MotorDriver(s.arm.can, can_id), joint)]
    elif choice.isdigit() and 1 <= int(choice) <= len(joints):
        joint = joints[int(choice) - 1]
        if joint.motor is None:
            print("  No board mapped to that joint (config.JOINTS).")
            return
        s.selected = [Target(joint.name, joint.motor, joint)]
    else:
        print("  Invalid choice.")
        return
    print(f"  Selected: {s.label}")


def submenu(s: Session, title: str, items: dict[str, tuple[str, Callable[[], object]]]) -> None:
    while True:
        print(f"\n  --- {title} ---")
        for key, (text, _) in items.items():
            print(f"  [{key}] {text}")
        print("  [b] Back")
        choice = input("  > ").strip().lower()
        if choice == "b":
            return
        if choice in items:
            items[choice][1]()
        else:
            print("  Invalid choice.")


def menu_read(s: Session) -> None:
    reads = [
        ("Read encoder (cumulative, 0x31)", cmd_read_encoder),
        ("Read encoder (carry, 0x30)", cmd_read_encoder_carry),
        ("Read speed RPM (0x32)", cmd_read_speed),
        ("Read motor status (0xF1)", cmd_read_status),
        ("Read enable status (0x3A)", cmd_read_enabled),
        ("Read shaft angle error (0x39)", cmd_read_angle_error),
        ("Read IO port status (0x34)", cmd_read_io),
        ("Read version (0x40)", cmd_read_version),
        ("Read voltage + current (0x38, V1.0.9)", cmd_read_power),
    ]
    submenu(s, "Read / Status", {str(i): (text, lambda f=f: s.run(f)) for i, (text, f) in enumerate(reads, 1)})


def ask_move(s: Session, what: str) -> tuple[int, int, int] | None:
    if not s.confirm_all():
        return None
    speed = input_int(f"Speed (RPM, 0-{MAX_SPEED_RPM})", config.DEFAULT_SPEED)
    if speed is None or not 0 <= speed <= MAX_SPEED_RPM or not warn_high_speed(speed):
        return None
    acc = input_int("Acceleration (0-255; 0 = no ramp)", config.DEFAULT_ACC)
    if acc is None or not 0 <= acc <= 255:
        return None
    value = input_int(what)
    return None if value is None else (speed, acc, value)


def menu_motion(s: Session) -> None:
    def absolute() -> None:
        if (move := ask_move(s, "Absolute position (encoder counts)")) is not None:
            s.run(cmd_absolute_move, *move)

    def relative() -> None:
        if (move := ask_move(s, "Relative position (encoder counts, negative = reverse)")) is not None:
            s.run(cmd_relative_move, *move)

    def speed_mode() -> None:
        if not s.confirm_all():
            return
        print("  Direction: 0=CCW, 1=CW (the manual's 0xF6)")
        direction = input_int("Direction", 0)
        if direction not in (0, 1):
            print("  Must be 0 or 1.")
            return
        speed = input_int(f"Speed (0-{MAX_SPEED_RPM})", config.DEFAULT_SPEED)
        if speed is None or not 0 <= speed <= MAX_SPEED_RPM or not warn_high_speed(speed):
            return
        acc = input_int("Acceleration (0-255)", config.DEFAULT_ACC)
        if acc is not None and 0 <= acc <= 255:
            s.run(cmd_speed_mode, direction, speed, acc)

    submenu(s, "Motion Commands", {
        "1": ("Absolute axis move (0xF5)", absolute),
        "2": ("Relative axis move (0xF4)", relative),
        "3": ("Speed mode (0xF6)", speed_mode),
        "4": ("Emergency stop the selected (0xF7)", lambda: s.run(cmd_emergency_stop, s.priority_estop)),
        "5": ("Emergency stop ALL (0xF7)", s.emergency_stop_all),
    })


def menu_homing(s: Session) -> None:
    def go_home() -> None:
        if not s.confirm_all():
            return
        raw = input("  goZeroMode: blank = none sent, 0 = return to zero, 1 = coordinate zeroing (V1.0.9): ").strip()
        if raw not in ("", "0", "1"):
            print("  Must be blank, 0 or 1.")
            return
        s.run(cmd_go_home, None if raw == "" else int(raw))

    def set_zero() -> None:
        if s.confirm_all():
            s.run(cmd_set_zero)

    def set_params() -> None:
        if not s.confirm_all():
            return
        print("  homeTrig: 0=low level when closed, 1=high level when closed")
        trig = input_int("homeTrig", 0)
        print("  homeDir: 0=CW, 1=CCW")
        hdir = input_int("homeDir", 0)
        hspeed = input_int("homeSpeed (RPM)", 30)
        print("  EndLimit: 0=off, 1=on")
        elimit = input_int("EndLimit", 0)
        raw = input("  hm_mode: blank = byte left out (firmware before it), 0 = origin switch, 1 = mechanical limit, 2 = single turn: ").strip()
        values = (trig, hdir, hspeed, elimit)
        if None in values or trig not in (0, 1) or hdir not in (0, 1) or elimit not in (0, 1) or raw not in ("", "0", "1", "2"):
            print("  Invalid value.")
            return
        s.run(cmd_set_home_params, trig, hdir, hspeed, elimit, None if raw == "" else int(raw))

    submenu(s, "Homing", {
        "1": ("Go home (0x91)", go_home),
        "2": ("Read home status (0x3B)", lambda: s.run(cmd_read_home_status)),
        "3": ("Set current position as zero (0x92)", set_zero),
        "4": ("Set home parameters (0x90)", set_params),
    })


def menu_enable(s: Session) -> None:
    def set_enabled(enable: bool) -> None:
        if not enable and not confirm_dangerous("A disabled motor holds nothing: a loaded joint falls. Support the arm."):
            return
        if s.confirm_all():
            s.run(cmd_enable, enable)

    submenu(s, "Enable / Disable", {
        "1": ("Enable motor", lambda: set_enabled(True)),
        "2": ("Disable motor", lambda: set_enabled(False)),
    })


def menu_config(s: Session) -> None:
    def ask(prompt: str, default: int | None, valid: Callable[[int], bool]) -> int | None:
        if not s.confirm_all():
            return None
        value = input_int(prompt, default)
        if value is None or not valid(value):
            print("  Invalid value.")
            return None
        return value

    def work_mode() -> None:
        print(f"  Modes: {WORK_MODES}")
        if (mode := ask("Mode", 5, lambda v: 0 <= v <= 5)) is not None:
            s.run(cmd_raw_status, bytes([Cmd.SET_MODE, mode]))

    def current() -> None:
        if (ma := ask("Current (mA, SERVO42D max 3000)", None, lambda v: 0 <= v <= 3000)) is not None:
            s.run(cmd_raw_status, bytes([Cmd.SET_CURRENT]) + encode_uint16(ma), (1, 2))

    def en_level() -> None:
        print("  0=Low, 1=High, 2=Hold (always enabled)")
        if (val := ask("EN active", 2, lambda v: v in (0, 1, 2))) is not None:
            s.run(cmd_raw_status, bytes([Cmd.SET_EN_LEVEL, val]))

    def stall_protect() -> None:
        print("  0=disable, 1=enable")
        if (val := ask("Stall protection", 1, lambda v: v in (0, 1))) is not None:
            s.run(cmd_raw_status, bytes([Cmd.SET_STALL_PROTECT, val]))

    def calibrate() -> None:
        if confirm_dangerous("Calibrate encoder. Motor must be UNLOADED (no belt/gearbox). This rotates the motor."):
            s.run(cmd_calibrate)

    def restore() -> None:
        if confirm_dangerous("Restore factory defaults (the CAN id and bitrate go back to theirs too)."):
            s.run(cmd_raw_status, bytes([Cmd.RESTORE_DEFAULTS]))

    def heartbeat() -> None:
        if (ms := ask("Heartbeat (ms without a command before the board stops; 0 = off)", 0, lambda v: 0 <= v <= 0xFFFFFFFF)) is not None:
            s.run(cmd_set_heartbeat, ms)

    def checksum_method() -> None:
        print("  0=SUM8 (default), 1=fixed 0x6B (V1.0.9)")
        if (val := ask("Checksum", 0, lambda v: v in (0, 1))) is not None:
            s.run(cmd_set_checksum, val)

    submenu(s, "Configuration", {
        "1": ("Set work mode (0x82)", work_mode),
        "2": ("Set working current (0x83)", current),
        "3": ("Set EN pin active (0x85)", en_level),
        "4": ("Set stall protection (0x88)", stall_protect),
        "5": ("Calibrate encoder (0x80)  [DANGEROUS]", calibrate),
        "6": ("Restore defaults (0x3F)   [DANGEROUS]", restore),
        "7": ("Restart motor (0x41)", lambda: s.confirm_all() and s.run(cmd_restart)),
        "8": ("Set heartbeat protection (0x98)", heartbeat),
        "9": ("Set checksum method (0x4C)", checksum_method),
    })


def menu_diagnostics(s: Session) -> None:
    submenu(s, "Diagnostics / Protection", {
        "1": ("Read protection status (0x3E)", lambda: s.run(cmd_read_stall)),
        "2": ("Release stall protection (0x3D)", lambda: s.confirm_all() and s.run(cmd_raw_status, bytes([Cmd.RELEASE_STALL]))),
    })


def main_menu(s: Session) -> None:
    menus = {
        "0": ("Select motor", menu_select),
        "1": ("Read / Status", menu_read),
        "2": ("Motion commands", menu_motion),
        "3": ("Homing", menu_homing),
        "4": ("Enable / Disable", menu_enable),
        "5": ("Configuration", menu_config),
        "6": ("Diagnostics / Protection", menu_diagnostics),
    }
    while True:
        print(f"\n{'=' * 42}\n  robotic-arm motor control\n  Selected: {s.label}\n{'=' * 42}")
        for key, (text, _) in menus.items():
            print(f"  [{key}] {text}")
        print("  [7] *** EMERGENCY STOP ALL ***")
        print("  [q] Quit")
        choice = input("> ").strip().lower()
        if choice == "q":
            return
        if choice == "7":
            s.emergency_stop_all()
        elif choice in menus:
            menus[choice][1](s)
        else:
            print("  Invalid choice.")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="motor-control", description="robotic-arm MKS motor control - interactive CLI")
    parser.add_argument("--channel", default=config.CAN_CHANNEL, help=f"CAN channel (default: {config.CAN_CHANNEL})")
    parser.add_argument("--interface", default=config.CAN_INTERFACE,
                        help=f"python-can interface: slcan, socketcan, virtual ... (default: {config.CAN_INTERFACE})")
    parser.add_argument("--bitrate", type=int, default=config.CAN_BITRATE,
                        help=f"CAN bitrate (default: {config.CAN_BITRATE}; socketcan takes its netdev's)")
    parser.add_argument("--priority-estop", action="store_true",
                        help="emergency stops in V1.0.9's immediate form (F7 01) - once the boards are known to run it")
    parser.add_argument("--fake", action="store_true", help="simulated boards on a virtual bus (implies --interface virtual)")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    fake = None
    if args.fake:
        from robotic_arm.fake import FakeBoards

        args.interface, args.channel = "virtual", "motor-control-fake"
        fake = FakeBoards(args.channel, [j.can_id for j in config.JOINTS if j.can_id is not None])
    bitrate = None if args.interface in ("socketcan", "virtual") else args.bitrate
    try:
        can = CANInterface(args.channel, args.interface, bitrate)
    except Exception as e:  # noqa: BLE001 - serial, python-can or OS: any failure to open ends the run
        print(f"Failed to open CAN bus: {e}")
        sys.exit(1)

    session = Session(Arm(can), args.priority_estop)

    def on_sigint(_signum: int, _frame: object) -> None:
        session.emergency_stop_all()
        sys.exit(1)

    previous = signal.signal(signal.SIGINT, on_sigint)
    print(f"Connected to {args.channel} ({args.interface})" + (f" at {bitrate} bps" if bitrate else ""))
    try:
        main_menu(session)
    finally:
        signal.signal(signal.SIGINT, previous)
        can.close()
        if fake is not None:
            fake.close()
        print("CAN bus closed.")


if __name__ == "__main__":
    main()
