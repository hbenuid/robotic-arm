"""Layer 1: raw CAN frames through python-can. Nothing here knows about MKS boards."""

from __future__ import annotations

import collections
import time
from collections.abc import Callable
from typing import Any, NamedTuple, Self

import can

from robotic_arm.config import CAN_BITRATE

PENDING_MAX = 1024   # frames kept for a later receive() before the oldest are dropped


class Frame(NamedTuple):
    """A received standard CAN frame."""

    can_id: int
    data: bytes
    timestamp: float   # python-can's receive timestamp, seconds - the interface's clock (socketcan: the kernel's)


class CANInterface:
    """One CAN bus through python-can: send frames, receive them with their timestamps.

    `interface` is python-can's name for the adapter's driver: "slcan" (a serial port), "socketcan" (a Linux CAN
    netdev such as can0; its bitrate is the netdev's, set with `ip link`), "virtual" (in-process, for tests).
    `bitrate=None` leaves the bitrate out of the bus's arguments; any other keyword goes to `can.Bus` as it is.

    A frame that one receive() passes over is kept, and a later receive() that wants it gets it (oldest first), so
    replies from several boards can be collected one at a time. At most PENDING_MAX are kept; the oldest go first.
    Not thread-safe: one thread per instance.
    """

    def __init__(self, channel: str, interface: str, bitrate: int | None = CAN_BITRATE, **kwargs: Any):
        if bitrate is not None:
            kwargs["bitrate"] = bitrate
        self._bus = can.Bus(channel=channel, interface=interface, **kwargs)
        self._pending: collections.deque[Frame] = collections.deque(maxlen=PENDING_MAX)

    def __enter__(self) -> Self:
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    def send(self, can_id: int, data: bytes) -> None:
        """Send a standard frame (11-bit id, up to 8 data bytes)."""
        if not 0 <= can_id <= 0x7FF:
            raise ValueError(f"standard CAN id out of range: {can_id:#x}")
        if len(data) > 8:
            raise ValueError(f"a CAN frame carries at most 8 data bytes, got {len(data)}")
        self._bus.send(can.Message(arbitration_id=can_id, data=bytes(data), is_extended_id=False))

    def receive(
        self,
        can_id: int | None = None,
        timeout: float = 1.0,
        match: Callable[[Frame], bool] | None = None,
    ) -> Frame | None:
        """The first frame from `can_id` (any id if None) that `match` accepts, or None after `timeout` seconds.

        Kept frames come first; frames read from the bus that are not wanted are kept for a later call.
        """

        def wanted(frame: Frame) -> bool:
            return (can_id is None or frame.can_id == can_id) and (match is None or match(frame))

        for i, frame in enumerate(self._pending):
            if wanted(frame):
                del self._pending[i]
                return frame
        deadline = time.monotonic() + timeout
        while True:
            msg = self._bus.recv(timeout=max(0.0, deadline - time.monotonic()))
            if msg is None:
                return None
            if not msg.is_error_frame and not msg.is_remote_frame:
                frame = Frame(msg.arbitration_id, bytes(msg.data), msg.timestamp)
                if wanted(frame):
                    return frame
                self._pending.append(frame)
            if time.monotonic() >= deadline:
                return None

    def discard(self, match: Callable[[Frame], bool]) -> int:
        """Drop the kept frames `match` accepts; returns how many."""
        keep = [f for f in self._pending if not match(f)]
        dropped = len(self._pending) - len(keep)
        self._pending.clear()
        self._pending.extend(keep)
        return dropped

    def drain(self) -> int:
        """Drop every kept frame and every frame waiting on the bus; returns how many."""
        dropped = len(self._pending)
        self._pending.clear()
        while self._bus.recv(timeout=0.0) is not None:
            dropped += 1
        return dropped

    def close(self) -> None:
        """Shut the bus down; a send after it raises can.CanOperationError."""
        self._bus.shutdown()
