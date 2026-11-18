"""Fake MKS boards on python-can's in-process "virtual" bus, for tests - no hardware, nothing moves.

    with FakeBoards("test-bus", [FakeBoard(0x01), FakeBoard(0x02)]) as fake, \\
            CANInterface("test-bus", "virtual", bitrate=None) as can:
        arm = Arm(can, joints)   # the joints' can_ids 1 and 2

The boards answer the commands motor_driver sends, in the bytes of the MKS manual V1.0.9, from a simple state model
(a move lands at once; it is not the firmware). A board with respond=False stays silent, like one that is missing.
`received` records every frame the boards saw, in order. The virtual bus reaches only buses in the same process,
opened on the same channel name; give each test its own name.
"""

from __future__ import annotations

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
    position: int = 0               # the 0x31 coordinate, counts
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


# Settings the fake only stores - each answers 1 (success), and 0x00 reads the value back.
SETTINGS = frozenset({0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8F, 0x94, 0x95, 0x9A,
                      0x9B, 0x9D, 0x9E, Cmd.SET_HOME_PARAMS})


def _int48(value: int) -> bytes:
    return value.to_bytes(6, "big", signed=True)


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
            for code, period in list(board.reports.items()):
                due = self._next_report.setdefault((board.can_id, code), now + period / 1000)
                if now >= due and board.respond:
                    self._next_report[(board.can_id, code)] = now + period / 1000
                    for body in self._answer(board, bytes([code])):
                        self._reply(board, body)

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
            carry, value = divmod(board.position, 0x4000)
            return [bytes([code]) + carry.to_bytes(4, "big", signed=True) + encode_uint16(value)]
        if code == Cmd.READ_ENCODER:
            return [bytes([code]) + _int48(board.position)]
        if code == Cmd.READ_SPEED:
            return [bytes([code]) + board.speed_rpm.to_bytes(2, "big", signed=True)]
        if code == Cmd.READ_IO:
            return [bytes([code, board.io])]
        if code == Cmd.READ_POWER:
            return [bytes([code]) + encode_uint16(board.millivolts) + encode_uint16(board.milliamps)]
        if code == Cmd.READ_ANGLE_ERROR:
            return [bytes([code, 0, 0, 0, 0])]
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
            board.position, board.home_status = 0, (*board.home_status[:-1], 1)
            return [bytes([code, 1]), bytes([code, 2])]
        if code == Cmd.SET_ZERO:
            board.position = 0
            return [bytes([code, 1])]
        if code == Cmd.READ_STATUS:
            return [bytes([code, board.status])]
        if code == Cmd.ENABLE and data:
            board.enabled = bool(data[0])
            return [bytes([code, 1])]
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
            board.speed_rpm, board.status = 0, 1
            return [bytes([code, 1])]
        return []


def frame_bytes(can_id: int, body: bytes, method: Checksum = Checksum.SUM8) -> bytes:
    """`body` with its check byte - for a test that writes a board's reply by hand."""
    return body + bytes([checksum(can_id, body, method)])

