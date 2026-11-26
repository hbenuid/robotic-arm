"""Fake MKS boards on python-can's in-process "virtual" bus, for tests - no hardware, nothing moves.

    with FakeBoards("test-bus", [FakeBoard(0x01), FakeBoard(0x02)]) as fake, \\
            CANInterface("test-bus", "virtual", bitrate=None) as can:
        arm = Arm(can, joints)   # the joints' can_ids 1 and 2

The boards answer the commands motor_driver sends, in the bytes of the MKS manual V1.0.9, from a simple state model
(a move lands at once; it is not the firmware). A board with respond=False stays silent, like one that is missing.
`received` records every frame the boards saw, in order. The virtual bus reaches only buses in the same process,
opened on the same channel name; give each test its own name.

A board with simulate=True moves in time instead: 0xF4 / 0xF5 / 0xF6 ramp the speed by the manual's acc rule (9.1: 1 RPM
every (256 - acc) x 50 us) up to the commanded RPM and brake onto a position target, answering 1 at the start and 2
when the motion ends (11.3 / 11.4: speed 0 is a stop, ramped or, at acc 0, at once); 0xF1, 0x32, 0x38 and 0x39 report
the motion. `switch_zone` puts a home switch on IN_1 (pulled up, read low inside the zone, as a KY-003 pulls it), in
counts of the shaft's own travel: 0x92 and homing move the coordinate's zero, not the switch. It is still not the
firmware - the dashboard's --fake and its tests use it.
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Iterable
from dataclasses import dataclass, field
from typing import Self

from robotic_arm.can_interface import CANInterface, Frame
from robotic_arm.motor_driver import Checksum, Cmd, checksum, encode_uint16


@dataclass
class FakeBoard:
    """One board's state; tests read and set it directly."""

    can_id: int
    respond: bool = True
    checksum: Checksum = Checksum.SUM8
    enabled: bool = False
    position: int = 0               # the 0xF4 / 0xF5 coordinate, counts
    encoder_offset: int = 0         # 0x30 / 0x31 read position + this: a real board reads its encoder's own angle at
                                    # power-on while F4 / F5 count from there, until 0x92 or homing (seen on V1.0.9)
    speed_rpm: int = 0              # signed, as 0x32 reports it
    status: int = 1                 # 0xF1: 1 stopped
    io: int = 0                     # 0x34
    stalled: bool = False
    calibrated: bool = True
    hardware: int = 2               # S42D_CAN
    firmware: bytes = b"\x01\x00\x09"
    bootloader: int = 0x20
    millivolts: int = 24_000
    milliamps: int = 0
    home_status: tuple[int, ...] = (1, 1)
    heartbeat_ms: int = 0
    heartbeat_tripped: bool = False
    params: dict[int, bytes] = field(default_factory=dict)    # 0x00 read-backs by code
    reports: dict[int, int] = field(default_factory=dict)     # 0x01: read code -> period ms
    last_seen: float = field(default_factory=time.monotonic)
    angle_error: int = 0            # 0x39
    # --- simulate=True only ---
    simulate: bool = False
    switch_zone: tuple[int, int] | None = None   # shaft counts (position + origin) where IN_1 reads low
    origin: int = 0                 # the shaft's travel at the coordinate's zero
    sub: float = 0.0                # the position's fraction of a count
    velocity: float = 0.0           # counts/s, signed
    target: int | None = None       # a position move's target; None: a speed run (run_rpm, 0 = stopping)
    run_rpm: int = 0                # a speed run's signed RPM
    move_rpm: float = 0.0           # a position move's RPM: its speed value x speed_factor()
    acc: int = 0
    done_code: int | None = None    # the motion's command code, answered 2 when it ends
    settle_short: int = 0           # a position move ends this many counts short (V1.0.8+'s arrival window, 0x95)
    sim_time: float = field(default_factory=time.monotonic)

    def speed_factor(self) -> float:
        """Real RPM per unit of a speed value at the microsteps set by 0x84 (9.1: 1 at 16 / 32 / 64, else 16 / m)."""
        m = self.params.get(0x84, b"\x10")[0]
        return 1.0 if m in (16, 32, 64) else 16 / m


# Settings the fake only stores - each answers 1 (success), and 0x00 reads the value back.
SETTINGS = frozenset({0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8F, 0x94, 0x95, 0x96,
                      0x97, 0x9A, 0x9B, 0x9D, 0x9E, Cmd.SET_HOME_PARAMS})


COUNTS_PER_RPM_S = 0x4000 / 60   # counts per second at 1 RPM


def _int48(value: int) -> bytes:
    return value.to_bytes(6, "big", signed=True)


def _accel(acc: int) -> float:
    """counts/s^2 for the ramp `acc` (9.1: 1 RPM every (256 - acc) x 50 us); acc 0 is no ramp."""
    return math.inf if acc == 0 else COUNTS_PER_RPM_S / ((256 - acc) * 50e-6)


def _halt(board: FakeBoard) -> None:
    board.velocity, board.target, board.run_rpm, board.speed_rpm, board.status = 0.0, None, 0, 0, 1


class FakeBoards:
    """A thread answering for `boards` (FakeBoard or bare CAN ids) on the virtual bus `channel`."""

    def __init__(self, channel: str, boards: Iterable[FakeBoard | int]):
        self.boards = {b.can_id: b for b in (x if isinstance(x, FakeBoard) else FakeBoard(x) for x in boards)}
        self.received: list[Frame] = []
        self._can = CANInterface(channel, "virtual", bitrate=None)
        self._stop = threading.Event()
        self._next_report: dict[tuple[int, int], float] = {}
        self._thread = threading.Thread(target=self._run, name=f"FakeBoards({channel})", daemon=True)
        self._thread.start()

    def __enter__(self) -> Self:
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
        self._can.close()

    def sent_to(self, can_id: int) -> list[bytes]:
        """The frames' bytes (check byte included) sent to `can_id` so far."""
        return [f.data for f in list(self.received) if f.can_id == can_id]

    # --- the loop ---------------------------------------------------------------------------------------------------

    def _run(self) -> None:
        while not self._stop.is_set():
            frame = self._can.receive(timeout=0.002)
            if frame is not None:
                self.received.append(frame)
                self._handle(frame)
            self._tick()

    def _reply(self, board: FakeBoard, body: bytes) -> None:
        self._can.send(board.can_id, body + bytes([checksum(board.can_id, body, board.checksum)]))

    def _tick(self) -> None:
        now = time.monotonic()
        for board in self.boards.values():
            if board.heartbeat_ms and (now - board.last_seen) * 1000 > board.heartbeat_ms and not board.heartbeat_tripped:
                board.heartbeat_tripped, board.speed_rpm, board.status = True, 0, 1
                _halt(board)
            if board.simulate:
                self._simulate(board, now)
            for code, period in list(board.reports.items()):
                due = self._next_report.setdefault((board.can_id, code), now + period / 1000)
                if now >= due and board.respond:
                    self._next_report[(board.can_id, code)] = now + period / 1000
                    for body in self._answer(board, bytes([code])):
                        self._reply(board, body)

    def _simulate(self, board: FakeBoard, now: float) -> None:
        """Advance a simulate=True board's motion to `now`."""
        dt, board.sim_time = min(0.05, now - board.sim_time), now
        if dt <= 0:
            return
        a = _accel(board.acc) * board.speed_factor()   # the ramp steps one speed value at a time
        pos = board.position + board.sub
        rem = 0.0
        if board.target is not None:
            rem = board.target - pos
            top = board.move_rpm * COUNTS_PER_RPM_S
            brake = top if a == math.inf else math.sqrt(2 * a * abs(rem))
            want = math.copysign(min(top, max(brake, 20.0)), rem) if rem else 0.0
        else:
            want = board.run_rpm * COUNTS_PER_RPM_S
        v = board.velocity
        v = want if abs(want - v) <= a * dt else v + math.copysign(a * dt, want - v)
        moved = v * dt
        if board.target is not None and (abs(rem) <= 1 or (moved * rem > 0 and abs(moved) >= abs(rem))):
            pos, v, board.target = float(board.target), 0.0, None
        else:
            pos += moved
        board.position = math.floor(pos)
        board.sub = pos - board.position
        board.velocity = v
        board.speed_rpm = round(v / COUNTS_PER_RPM_S)
        if v == 0.0 and board.target is None and board.run_rpm == 0:
            board.status = 1
            if board.done_code is not None:
                code, board.done_code = board.done_code, None
                if board.respond:
                    self._reply(board, bytes([code, 2]))
        else:
            board.status = 2 if abs(v) < abs(want) - 1 else 3 if abs(v) > abs(want) + 1 else 4
        rpm = abs(v) / COUNTS_PER_RPM_S
        board.milliamps = round(250 + 0.6 * rpm + (400 if board.status in (2, 3) else 0))
        board.angle_error = round(v / COUNTS_PER_RPM_S * 4)
        if board.switch_zone is not None:
            lo, hi = board.switch_zone
            board.io = (board.io & ~1) | (0 if lo <= board.position + board.origin <= hi else 1)

    def _simulated_motion(self, board: FakeBoard, code: int, data: bytes) -> list[bytes]:
        """0xF4 / 0xF5 / 0xF6 on a simulate=True board: start the motion; the tick answers 2 when it ends."""
        if not board.enabled:
            return [bytes([code, 0])]
        f = board.speed_factor()
        if code == Cmd.SPEED:
            speed, board.acc = ((data[0] & 0x0F) << 8) | data[1], data[2]
            board.target, board.run_rpm = None, round((-speed if data[0] & 0x80 else speed) * f)
            board.done_code = code if speed == 0 else None
        else:
            speed, board.acc = int.from_bytes(data[0:2], "big"), data[2]
            value = int.from_bytes(data[3:6], "big", signed=True)
            if speed == 0:   # 11.3.2 / 11.4.2: the stop
                board.target, board.run_rpm = None, 0
            else:
                target = value if code == Cmd.MOVE_TO_AXIS else board.position + value
                travel = target - board.position
                if board.settle_short:   # inside the window it stays put; from outside it stops at its edge
                    target = board.position if abs(travel) <= board.settle_short else \
                        target - (board.settle_short if travel > 0 else -board.settle_short)
                board.target = target
                board.move_rpm, board.run_rpm = min(speed, 3000) * f, 0
            board.done_code = code
        if board.acc == 0 and board.target is None and board.run_rpm == 0:
            board.velocity = 0.0
        board.sim_time = time.monotonic()
        return [bytes([code, 1])]

    def _handle(self, frame: Frame) -> None:
        board = self.boards.get(frame.can_id)
        if board is None or len(frame.data) < 2:
            return
        body, check = frame.data[:-1], frame.data[-1]
        if check != checksum(board.can_id, body, board.checksum):
            return   # a bad check byte: ignored, as a board does
        board.last_seen = time.monotonic()
        if not board.respond:
            return
        for reply in self._answer(board, body):
            self._reply(board, reply)

    def _answer(self, board: FakeBoard, body: bytes) -> list[bytes]:
        """The board's replies to `body`, one case per code; none for a code it does not model."""
        code, data = body[0], body[1:]
        if code in SETTINGS and data:
            board.params[code] = bytes(data)
            return [bytes([code, 1])]
        if code == Cmd.CALIBRATE:
            return [bytes([code, 0]), bytes([code, 1])]
        if code in (Cmd.RESTORE_DEFAULTS, Cmd.RESTART):
            return [bytes([code, 1])]
        if code == Cmd.READ_PARAM and data:
            return [bytes([data[0]]) + board.params.get(data[0], b"\xff\xff")]
        if code == Cmd.AUTO_REPORT and len(data) == 3:
            period = int.from_bytes(data[1:3], "big")
            if period:
                board.reports[data[0]] = period
            else:
                board.reports.pop(data[0], None)
            self._next_report.pop((board.can_id, data[0]), None)
            return [bytes([Cmd.AUTO_REPORT, data[0], 1])]
        if code == Cmd.READ_ENCODER_CARRY:
            carry, value = divmod(board.position + board.encoder_offset, 0x4000)
            return [bytes([code]) + carry.to_bytes(4, "big", signed=True) + encode_uint16(value)]
        if code == Cmd.READ_ENCODER:
            return [bytes([code]) + _int48(board.position + board.encoder_offset)]
        if code == Cmd.READ_SPEED:
            return [bytes([code]) + board.speed_rpm.to_bytes(2, "big", signed=True)]
        if code == Cmd.READ_IO:
            return [bytes([code, board.io])]
        if code == Cmd.READ_POWER:
            return [bytes([code]) + encode_uint16(board.millivolts) + encode_uint16(board.milliamps)]
        if code == Cmd.READ_ANGLE_ERROR:
            return [bytes([code]) + board.angle_error.to_bytes(4, "big", signed=True)]
        if code == Cmd.READ_ENABLED:
            return [bytes([code, int(board.enabled)])]
        if code == Cmd.READ_HOME_STATUS:
            return [bytes([code, *board.home_status])]
        if code == Cmd.RELEASE_STALL:
            board.stalled = False
            return [bytes([code, 1])]
        if code == Cmd.READ_STALL:
            return [bytes([code, int(board.stalled)])]
        if code == Cmd.READ_VERSION:
            return [bytes([code, (int(board.calibrated) << 4) | board.hardware]) + board.firmware + bytes([board.bootloader])]
        if code == Cmd.SET_CHECKSUM and data:
            board.checksum = Checksum(data[0])
            return [bytes([code, 1])]
        if code == Cmd.SET_HEARTBEAT and len(data) == 4:
            board.heartbeat_ms, board.heartbeat_tripped = int.from_bytes(data, "big"), False
            return [bytes([code, 1])]
        if code == Cmd.GO_HOME:
            if board.simulate:   # lands on the switch's edge at once; that becomes the coordinate's zero
                _halt(board)
                edge = board.switch_zone[1] if board.switch_zone is not None else board.position + board.origin
                board.origin, board.sub = edge, 0.0
            board.position, board.encoder_offset, board.home_status = 0, 0, (*board.home_status[:-1], 1)
            return [bytes([code, 1]), bytes([code, 2])]
        if code == Cmd.SET_ZERO:
            board.origin += board.position
            board.position, board.sub, board.encoder_offset = 0, 0.0, 0
            return [bytes([code, 1])]
        if code == Cmd.READ_STATUS:
            return [bytes([code, board.status])]
        if code == Cmd.ENABLE and data:
            board.enabled = bool(data[0])
            if board.simulate and not board.enabled:
                _halt(board)
            return [bytes([code, 1])]
        if board.simulate and ((code in (Cmd.MOVE_TO_AXIS, Cmd.MOVE_RELATIVE_AXIS) and len(data) == 6)
                               or (code == Cmd.SPEED and len(data) == 3)):
            return self._simulated_motion(board, code, data)
        if code in (Cmd.MOVE_TO_AXIS, Cmd.MOVE_RELATIVE_AXIS) and len(data) == 6:
            if not board.enabled:
                return [bytes([code, 0])]
            target = int.from_bytes(data[3:6], "big", signed=True)
            board.position = target if code == Cmd.MOVE_TO_AXIS else board.position + target
            return [bytes([code, 1]), bytes([code, 2])]
        if code == Cmd.SPEED and len(data) == 3:
            if not board.enabled:
                return [bytes([code, 0])]
            speed = ((data[0] & 0x0F) << 8) | data[1]
            board.speed_rpm = -speed if data[0] & 0x80 else speed
            board.status = 4 if speed else 1
            return [bytes([code, 1 if speed else 2])]
        if code == Cmd.EMERGENCY_STOP:
            _halt(board)
            board.done_code = None
            return [bytes([code, 1])]
        return []


def frame_bytes(can_id: int, body: bytes, method: Checksum = Checksum.SUM8) -> bytes:
    """`body` with its check byte - for a test that writes a board's reply by hand."""
    return body + bytes([checksum(can_id, body, method)])

