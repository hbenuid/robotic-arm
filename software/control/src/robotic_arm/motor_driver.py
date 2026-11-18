"""Layer 2: the MKS SERVO42D/57D CAN protocol - command frames, checksums, replies. One MotorDriver per board.

The bytes are the MKS SERVO42D/57D CAN manual V1.0.9's (its section numbers below; the reference is
docs/mks_can_protocol.md). Which firmware the boards run is not known yet (cad/docs/open_issues.md): where the manual's
version notes date a command or a change, the docstring says so, and read_version() (0x40) or the board's About menu
tells which firmware a board runs. Gear ratios, joint angles and limits are not this layer's: joint.py, arm.py.
"""

from __future__ import annotations

import time
from enum import IntEnum
from typing import NamedTuple

from robotic_arm.can_interface import CANInterface, Frame
from robotic_arm.config import REPLY_TIMEOUT_S

MAX_SPEED_RPM = 3000          # 9.1: speed 0-3000; above a work mode's own maximum the board runs at that maximum
MAX_ACC = 255                 # 9.1: acc 0-255; 0 = no ramp, straight to the set speed
INT24_MAX = 8_388_607         # 11.3 / 11.4: the axis coordinates' range, +/-
FIXED_CHECKSUM_BYTE = 0x6B    # Part 4 / 5.2.13


class Checksum(IntEnum):
    """The frame check byte (Part 4; the CanCRC menu, 0x4C)."""

    SUM8 = 0    # (CAN id + every byte before it) & 0xFF - the factory default
    FIXED = 1   # always 0x6B


class Cmd(IntEnum):
    """The command codes this package sends."""

    READ_PARAM = 0x00            # 5.1.10
    AUTO_REPORT = 0x01           # 5.1.9
    READ_ENCODER_CARRY = 0x30    # 5.1.1
    READ_ENCODER = 0x31          # 5.1.2
    READ_SPEED = 0x32            # 5.1.3
    READ_IO = 0x34               # 6.1
    READ_POWER = 0x38            # 5.1.13
    READ_ANGLE_ERROR = 0x39      # 5.1.6
    READ_ENABLED = 0x3A          # 5.1.7
    READ_HOME_STATUS = 0x3B      # 5.1.8
    RELEASE_STALL = 0x3D         # 5.4.4
    READ_STALL = 0x3E            # 5.4.3
    RESTORE_DEFAULTS = 0x3F      # 5.5.1
    READ_VERSION = 0x40          # 5.1.11
    RESTART = 0x41               # 5.5.2
    SET_CHECKSUM = 0x4C          # 5.2.13
    CALIBRATE = 0x80             # 5.2.1
    SET_MODE = 0x82              # 5.2.2
    SET_CURRENT = 0x83           # 5.2.3
    SET_EN_LEVEL = 0x85          # 5.2.6
    SET_STALL_PROTECT = 0x88     # 5.4.1
    SET_HOME_PARAMS = 0x90       # 7.3.2
    GO_HOME = 0x91               # 7.3.6
    SET_ZERO = 0x92              # 7.3.5
    SET_HEARTBEAT = 0x98         # 5.2.16
    READ_STATUS = 0xF1           # 9.2.1
    ENABLE = 0xF3                # 9.2.2
    MOVE_RELATIVE_AXIS = 0xF4    # 11.3
    MOVE_TO_AXIS = 0xF5          # 11.4
    SPEED = 0xF6                 # 10.1 / 10.2
    EMERGENCY_STOP = 0xF7        # 9.2.3


STATUS_NAMES = {0: "query failed", 1: "stopped", 2: "speeding up", 3: "slowing down", 4: "full speed",
                5: "homing", 6: "calibrating"}   # 0xF1, 9.2.1
HOME_STATUS_NAMES = {0: "going", 1: "success", 2: "failed"}   # 0x3B, 5.1.8
HARDWARE_NAMES = {1: "S42D_485", 2: "S42D_CAN", 3: "S57D_485", 4: "S57D_CAN", 5: "S28D_RS485", 6: "S28D_CAN",
                  7: "S35D_RS485", 8: "S35D_CAN"}   # 0x40, 5.1.11
PARAM_UNSUPPORTED = b"\xff\xff"   # 0x00's reply for a parameter that cannot be read back (5.1.10)


def checksum(can_id: int, data: bytes, method: Checksum = Checksum.SUM8) -> int:
    """The frame's last byte for `data` (the code and its data) sent to or from `can_id`."""
    if method is Checksum.FIXED:
        return FIXED_CHECKSUM_BYTE
    return (can_id + sum(data)) & 0xFF


def encode_uint16(value: int) -> bytes:
    return value.to_bytes(2, "big")


def encode_uint32(value: int) -> bytes:
    return value.to_bytes(4, "big")


def encode_int24(value: int) -> bytes:
    return value.to_bytes(3, "big", signed=True)


def decode_int16(data: bytes) -> int:
    return int.from_bytes(data[:2], "big", signed=True)


def decode_int32(data: bytes) -> int:
    return int.from_bytes(data[:4], "big", signed=True)


def decode_int48(data: bytes) -> int:
    return int.from_bytes(data[:6], "big", signed=True)


def _check_range(name: str, value: int, lo: int, hi: int) -> None:
    if not lo <= value <= hi:
        raise ValueError(f"{name} {value} outside {lo}..{hi}")


def move_to_axis_request(position: int, speed: int, acc: int) -> bytes:
    """0xF5 (11.4.1): to the absolute coordinate `position` (0x31 counts) at `speed` RPM, ramp `acc`."""
    _check_range("speed", speed, 0, MAX_SPEED_RPM)
    _check_range("acc", acc, 0, MAX_ACC)
    _check_range("position", position, -INT24_MAX, INT24_MAX)
    return bytes([Cmd.MOVE_TO_AXIS]) + encode_uint16(speed) + bytes([acc]) + encode_int24(position)


def move_relative_axis_request(offset: int, speed: int, acc: int) -> bytes:
    """0xF4 (11.3.1): by `offset` counts from where the board is."""
    _check_range("speed", speed, 0, MAX_SPEED_RPM)
    _check_range("acc", acc, 0, MAX_ACC)
    _check_range("offset", offset, -INT24_MAX, INT24_MAX)
    return bytes([Cmd.MOVE_RELATIVE_AXIS]) + encode_uint16(speed) + bytes([acc]) + encode_int24(offset)


def speed_request(direction: int, speed: int, acc: int) -> bytes:
    """0xF6 (10.1): run at `speed` RPM; direction 0 or 1 (the manual: "0/1 (CCW/CW)"); speed 0 stops (10.2)."""
    _check_range("direction", direction, 0, 1)
    _check_range("speed", speed, 0, MAX_SPEED_RPM)
    _check_range("acc", acc, 0, MAX_ACC)
    return bytes([Cmd.SPEED, (direction << 7) | (speed >> 8), speed & 0xFF, acc])


def emergency_stop_request(priority: bool = False) -> bytes:
    """0xF7 (9.2.3). The plain form "may fail to stop the unit if the current is too low"; `priority` adds the 01
    byte of V1.0.9's form that "shuts down immediately, regardless of current magnitude" (V1.0.9's notes: "Modified
    the F7 command"). Older firmware's handling of that byte is not in the manual."""
    return bytes([Cmd.EMERGENCY_STOP, 0x01]) if priority else bytes([Cmd.EMERGENCY_STOP])


class Reply(NamedTuple):
    """A board's checked reply."""

    can_id: int
    data: bytes        # the frame without its checksum byte: data[0] is the code
    timestamp: float   # the frame's python-can receive timestamp

    @property
    def code(self) -> int:
        return self.data[0]

    @property
    def status(self) -> int | None:
        """The byte after the code - most commands' status - or None if the reply has none."""
        return self.data[1] if len(self.data) > 1 else None


class Version(NamedTuple):
    """0x40's reply (5.1.11)."""

    calibrated: bool
    hardware: int      # HARDWARE_NAMES: 2 = S42D_CAN
    firmware: bytes    # every byte between Byte2 and the boot version, as the board sent them
    bootloader: int    # 0x10: upgraded over RS485, 0x20: over CAN

    @property
    def firmware_text(self) -> str:
        return ".".join(str(b) for b in self.firmware)


class Power(NamedTuple):
    """0x38's reply (5.1.13)."""

    millivolts: int
    milliamps: int


def decode_version(reply: Reply) -> Version | None:
    """The manual's table gives the firmware version as "Bytes 3-4 ... firmVer[3]" under DLC 5 with a Byte6: it is
    ambiguous, so this takes every byte between Byte2 and the last data byte as the firmware version."""
    d = reply.data
    if len(d) < 4:
        return None
    return Version(bool(d[1] >> 4), d[1] & 0x0F, bytes(d[2:-1]), d[-1])


def decode_power(reply: Reply) -> Power | None:
    d = reply.data
    if len(d) < 5:
        return None
    return Power(int.from_bytes(d[1:3], "big"), int.from_bytes(d[3:5], "big"))


def decode_encoder(reply: Reply) -> int | None:
    return decode_int48(reply.data[1:7]) if len(reply.data) >= 7 else None


class MotorDriver:
    """One MKS SERVO42D/57D board on the bus: builds its frames, sends them, checks and parses its replies.

    Each blocking method sends a request and waits `timeout` seconds for the reply with the request's code; it
    returns None (a read) or False (a setting or a move) when no checked reply comes. send() and parse() are the
    non-blocking pair: send to several boards, then collect their replies (arm.Arm.collect).
    A board replies only to its own CAN id (Part 5 note 1: none to the broadcast id 0 or a group id).
    """

    def __init__(self, can: CANInterface, can_id: int, checksum: Checksum = Checksum.SUM8):
        _check_range("CAN id", can_id, 1, 0x7FF)
        self._can = can
        self._can_id = can_id
        self._checksum = checksum

    @property
    def can_id(self) -> int:
        return self._can_id

    @property
    def checksum(self) -> Checksum:
        return self._checksum

    # --- frames ----------------------------------------------------------------------------------------------------

    def encode(self, data: bytes) -> bytes:
        """`data` (the code and its data) with the check byte appended."""
        if not 1 <= len(data) <= 7:
            raise ValueError(f"an MKS command is 1-7 bytes before its checksum, got {len(data)}")
        return bytes(data) + bytes([checksum(self._can_id, data, self._checksum)])

    def send(self, data: bytes) -> None:
        """Send a command without waiting for its reply."""
        self._can.send(self._can_id, self.encode(data))

    def parse(self, frame: Frame, methods: tuple[Checksum, ...] | None = None) -> Reply | None:
        """`frame` as this board's reply, or None: another id, too short, or a wrong check byte."""
        if frame.can_id != self._can_id or len(frame.data) < 2:
            return None
        body, check = frame.data[:-1], frame.data[-1]
        if all(check != checksum(self._can_id, body, m) for m in methods or (self._checksum,)):
            return None
        return Reply(frame.can_id, body, frame.timestamp)

    def request(
        self,
        data: bytes,
        timeout: float = REPLY_TIMEOUT_S,
        *,
        code: int | None = None,
        methods: tuple[Checksum, ...] | None = None,
    ) -> Reply | None:
        """Send `data` and wait for this board's reply whose code is `code` (default: the request's own).

        Kept replies with that code from before the request are dropped first: they answer an earlier one.
        """
        code = data[0] if code is None else code

        def is_reply(frame: Frame) -> bool:
            reply = self.parse(frame, methods)
            return reply is not None and reply.code == code

        self._can.discard(is_reply)
        self.send(data)
        frame = self._can.receive(self._can_id, timeout, match=is_reply)
        return None if frame is None else self.parse(frame, methods)

    def wait_reply(self, code: int, timeout: float) -> Reply | None:
        """This board's next reply with `code`, sending nothing - a move's second reply when it ends, say."""
        frame = self._can.receive(self._can_id, timeout, match=lambda f: (r := self.parse(f)) is not None and r.code == code)
        return None if frame is None else self.parse(frame)

    def raw_command(self, data: bytes, timeout: float = REPLY_TIMEOUT_S) -> bytes | None:
        """Send `data` (no check byte) and return the reply's bytes without the check byte, data[0] the code."""
        reply = self.request(data, timeout)
        return None if reply is None else reply.data

    def _status(self, data: bytes, timeout: float = REPLY_TIMEOUT_S) -> int | None:
        reply = self.request(data, timeout)
        return None if reply is None else reply.status

    # --- reads -----------------------------------------------------------------------------------------------------

    def read_encoder(self, timeout: float = REPLY_TIMEOUT_S) -> int | None:
        """0x31 (5.1.2): the cumulative encoder value, int48, 0x4000 per motor turn - the 0xF4 / 0xF5 coordinate."""
        reply = self.request(bytes([Cmd.READ_ENCODER]), timeout)
        return None if reply is None else decode_encoder(reply)

    def read_encoder_carry(self, timeout: float = REPLY_TIMEOUT_S) -> tuple[int, int] | None:
        """0x30 (5.1.1): (carry, value) - turns (int32) and the angle in the turn (0-0x3FFF)."""
        reply = self.request(bytes([Cmd.READ_ENCODER_CARRY]), timeout)
        if reply is None or len(reply.data) < 7:
            return None
        return decode_int32(reply.data[1:5]), int.from_bytes(reply.data[5:7], "big")

    def read_speed(self, timeout: float = REPLY_TIMEOUT_S) -> int | None:
        """0x32 (5.1.3): RPM, int16 - positive turning CCW, negative CW."""
        reply = self.request(bytes([Cmd.READ_SPEED]), timeout)
        if reply is None or len(reply.data) < 3:
            return None
        return decode_int16(reply.data[1:3])

    def read_io(self, timeout: float = REPLY_TIMEOUT_S) -> int | None:
        """0x34 (6.1): bit0 IN_1, bit1 IN_2, bit2 OUT_1, bit3 OUT_2 (remapped by 0x9E: bit0 En, bit1 Dir)."""
        return self._status(bytes([Cmd.READ_IO]), timeout)

    def read_power(self, timeout: float = REPLY_TIMEOUT_S) -> Power | None:
        """0x38 (5.1.13): the supply's mV and the motor's mA. Added in V1.0.9."""
        reply = self.request(bytes([Cmd.READ_POWER]), timeout)
        return None if reply is None else decode_power(reply)

    def read_angle_error(self, timeout: float = REPLY_TIMEOUT_S) -> int | None:
        """0x39 (5.1.6): commanded minus actual angle, int32, 51200 = 360 deg of the motor."""
        reply = self.request(bytes([Cmd.READ_ANGLE_ERROR]), timeout)
        if reply is None or len(reply.data) < 5:
            return None
        return decode_int32(reply.data[1:5])

    def read_enabled(self, timeout: float = REPLY_TIMEOUT_S) -> bool | None:
        """0x3A (5.1.7)."""
        status = self._status(bytes([Cmd.READ_ENABLED]), timeout)
        return None if status is None else status == 1

    def read_home_status(self, timeout: float = REPLY_TIMEOUT_S) -> tuple[int, ...] | None:
        """0x3B (5.1.8): the status bytes as sent (HOME_STATUS_NAMES). V1.0.9 sends two - the single-turn zero
        return's, then the homing's (0x91) - since V1.0.7 ("The 3BH instruction adds the ability to read the
        zero-state value"); what an earlier firmware's fewer bytes mean is not in the manual."""
        reply = self.request(bytes([Cmd.READ_HOME_STATUS]), timeout)
        if reply is None or len(reply.data) < 2:
            return None
        return tuple(reply.data[1:])

    def read_stalled(self, timeout: float = REPLY_TIMEOUT_S) -> bool | None:
        """0x3E (5.4.3)."""
        status = self._status(bytes([Cmd.READ_STALL]), timeout)
        return None if status is None else status == 1

    def read_version(self, timeout: float = REPLY_TIMEOUT_S) -> Version | None:
        """0x40 (5.1.11): calibrated, board type, firmware and bootloader versions."""
        reply = self.request(bytes([Cmd.READ_VERSION]), timeout)
        return None if reply is None else decode_version(reply)

    def read_status(self, timeout: float = REPLY_TIMEOUT_S) -> int | None:
        """0xF1 (9.2.1): STATUS_NAMES. Bus control modes only (SR_OPEN / SR_CLOSE / SR_vFOC)."""
        return self._status(bytes([Cmd.READ_STATUS]), timeout)

    def read_param(self, code: int, timeout: float = REPLY_TIMEOUT_S) -> bytes | None:
        """0x00 (5.1.10): a setting's value as it was written with `code` (0x82 the work mode, 0x83 the current ...);
        PARAM_UNSUPPORTED for one that cannot be read back. The reply carries `code`, not 0x00."""
        reply = self.request(bytes([Cmd.READ_PARAM, code]), timeout, code=code)
        return None if reply is None else bytes(reply.data[1:])

    # --- settings --------------------------------------------------------------------------------------------------

    def set_auto_report(self, code: int, period_ms: int, timeout: float = REPLY_TIMEOUT_S) -> bool:
        """0x01 (5.1.9): the board sends the read `code`'s reply by itself every `period_ms` (0 stops it). The
        reports are frames like the read's own replies. Added in V1.0.9."""
        _check_range("period_ms", period_ms, 0, 0xFFFF)
        reply = self.request(bytes([Cmd.AUTO_REPORT, code]) + encode_uint16(period_ms), timeout)
        return reply is not None and reply.data[1:3] == bytes([code, 1])

    def set_checksum_method(self, method: Checksum, timeout: float = REPLY_TIMEOUT_S) -> bool:
        """0x4C (5.2.13): the board's check byte from now on; this driver follows on success. The manual does not say
        which method the reply itself uses, so either is accepted. Added in V1.0.9."""
        reply = self.request(bytes([Cmd.SET_CHECKSUM, method]), timeout, methods=tuple(Checksum))
        ok = reply is not None and reply.status == 1
        if ok:
            self._checksum = Checksum(method)
        return ok

    def set_heartbeat(self, period_ms: int, timeout: float = REPLY_TIMEOUT_S) -> bool:
        """0x98 (5.2.16): the board stops urgently when no command reaches it for `period_ms` (0, the default: off).
        V1.0.9's notes "Increase the heart rate protection time"."""
        _check_range("period_ms", period_ms, 0, 0xFFFF_FFFF)
        return self._status(bytes([Cmd.SET_HEARTBEAT]) + encode_uint32(period_ms), timeout) == 1

    def set_home_params(self, trigger: int, direction: int, speed: int, end_limit: bool, mode: int | None = None,
                        timeout: float = REPLY_TIMEOUT_S) -> bool:
        """0x90 (7.3.2): the switch's closed level (0 low, 1 high), the homing direction (0 CW, 1 CCW), its RPM,
        the end limit on/off, and V1.0.9's hm_mode (0 origin switch, 1 mechanical limit, 2 single turn). `mode=None`
        leaves the hm_mode byte out, as manuals before it did."""
        _check_range("trigger", trigger, 0, 1)
        _check_range("direction", direction, 0, 1)
        _check_range("speed", speed, 0, MAX_SPEED_RPM)
        data = bytes([Cmd.SET_HOME_PARAMS, trigger, direction]) + encode_uint16(speed) + bytes([int(end_limit)])
        if mode is not None:
            _check_range("mode", mode, 0, 2)
            data += bytes([mode])
        return self._status(data, timeout) == 1

    def calibrate(self, timeout: float = 30.0) -> int | None:
        """0x80 (5.2.1): calibrate the encoder - the motor unloaded, it turns. Waits through the board's 0
        (calibrating) for 1 (success) or 2 (failed); returns the last status, None if no reply came."""
        deadline = time.monotonic() + timeout
        reply = self.request(bytes([Cmd.CALIBRATE, 0x00]), timeout)
        status = None
        while reply is not None:
            status = reply.status
            if status != 0:
                break
            reply = self.wait_reply(Cmd.CALIBRATE, max(0.0, deadline - time.monotonic()))
        return status

    def release_stall(self, timeout: float = REPLY_TIMEOUT_S) -> bool:
        """0x3D (5.4.4)."""
        return self._status(bytes([Cmd.RELEASE_STALL]), timeout) == 1

    # --- motion ----------------------------------------------------------------------------------------------------

    def enable(self, enabled: bool = True, timeout: float = REPLY_TIMEOUT_S) -> bool:
        """0xF3 (9.2.2): lock (True) or free (False) the shaft. A freed joint is not held: it falls if loaded."""
        return self._status(bytes([Cmd.ENABLE, int(enabled)]), timeout) == 1

    def move_to_axis(self, position: int, speed: int, acc: int, timeout: float = REPLY_TIMEOUT_S) -> bool:
        """0xF5 (11.4): True when the board answers 1 (started) or 2 (complete). In the default response mode
        (0x8C) a second reply follows when the move ends (2, or 3: stopped at a limit). The coordinate's zero is
        where homing (0x91) or 0x92 put it - or the power-on position if neither ran."""
        return self._status(move_to_axis_request(position, speed, acc), timeout) in (1, 2)

    def move_relative_axis(self, offset: int, speed: int, acc: int, timeout: float = REPLY_TIMEOUT_S) -> bool:
        """0xF4 (11.3): replies as move_to_axis."""
        return self._status(move_relative_axis_request(offset, speed, acc), timeout) in (1, 2)

    def set_speed(self, direction: int, speed: int, acc: int, timeout: float = REPLY_TIMEOUT_S) -> bool:
        """0xF6 (10.1): True when the board answers 1 (running) or 2 (complete)."""
        return self._status(speed_request(direction, speed, acc), timeout) in (1, 2)

    def stop(self, acc: int = 0, timeout: float = REPLY_TIMEOUT_S) -> bool:
        """0xF6 with speed 0 (10.2): acc 0 stops at once (not above 1000 RPM, the manual warns), acc > 0 ramps down."""
        return self._status(speed_request(0, 0, acc), timeout) in (1, 2)

    def emergency_stop(self, priority: bool = False, timeout: float = REPLY_TIMEOUT_S) -> bool:
        """0xF7 (9.2.3): True when the board confirms (1). `priority`: emergency_stop_request."""
        return self._status(emergency_stop_request(priority), timeout) == 1

    def go_home(self, mode: int | None = None, timeout: float = REPLY_TIMEOUT_S) -> int | None:
        """0x91 (7.3.6): start homing; returns the board's status - V1.0.9: 0 failed, 1 started, 2 complete (in the
        default response mode a second reply follows; poll read_home_status()). `mode` is V1.0.9's goZeroMode byte
        (0 return to zero, 1 coordinate zeroing); None sends none, which V1.0.9 takes as 0."""
        data = bytes([Cmd.GO_HOME])
        if mode is not None:
            _check_range("mode", mode, 0, 1)
            data += bytes([mode])
        return self._status(data, timeout)

    def set_zero(self, timeout: float = REPLY_TIMEOUT_S) -> bool:
        """0x92 (7.3.5): the current position becomes the coordinate zero, without moving."""
        return self._status(bytes([Cmd.SET_ZERO]), timeout) == 1

    def wait_stopped(self, timeout: float, poll_s: float = 0.1) -> bool:
        """Poll 0xF1 until the board reports stopped (1); False on timeout or a failed query."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = self.read_status(timeout=min(REPLY_TIMEOUT_S, max(0.05, deadline - time.monotonic())))
            if status == 1:
                return True
            time.sleep(poll_s)
        return False
