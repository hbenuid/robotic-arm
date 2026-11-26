"""Layer 4: the arm's joints on one bus - send to every board, then collect their replies.

Every multi-board operation sends all its frames first and only then waits, so a board that does not answer delays
no other board's command. Homing order, waiting for moves, trajectories: the plan in docs/architecture.md.
"""

from __future__ import annotations

import time
from collections.abc import Callable, Iterable, Mapping

from can import CanError

from robotic_arm.can_interface import CANInterface, Frame
from robotic_arm.config import JOINTS, REPLY_TIMEOUT_S, JointConfig
from robotic_arm.joint import Joint, JointLimits, JointSample
from robotic_arm.motor_driver import (
    Checksum,
    Cmd,
    MotorDriver,
    Reply,
    decode_encoder,
    emergency_stop_request,
    move_to_axis_request,
)

ESTOP_TIMEOUT_S = 0.25   # how long emergency_stop() waits for the boards' confirmations, after sending every stop


class Arm:
    """The joints of `joints` (default: config.JOINTS) on `can`; a joint whose can_id is None has no board."""

    def __init__(self, can: CANInterface, joints: Iterable[JointConfig] = JOINTS, checksum: Checksum = Checksum.SUM8):
        self.can = can
        self.joints: dict[str, Joint] = {}
        for config in joints:
            motor = None if config.can_id is None else MotorDriver(can, config.can_id, checksum)
            self.joints[config.name] = Joint(config, motor)
        ids = [j.config.can_id for j in self.mapped]
        if len(ids) != len(set(ids)):
            raise ValueError(f"two joints share a CAN id: {ids}")

    @property
    def mapped(self) -> list[Joint]:
        """The joints with a board, in table order."""
        return [j for j in self.joints.values() if j.motor is not None]

    def _targets(self, names: Iterable[str] | None) -> list[Joint]:
        if names is None:
            return self.mapped
        targets = [self.joints[n] for n in names]
        unmapped = [j.name for j in targets if j.motor is None]
        if unmapped:
            raise ValueError(f"no board mapped to {unmapped}")
        return targets

    def send_all(self, data: bytes, names: Iterable[str] | None = None) -> list[str]:
        """Send `data` to each joint's board (default: every mapped one) without waiting; returns their names."""
        targets = self._targets(names)
        for joint in targets:
            joint.board.send(data)
        return [j.name for j in targets]

    def collect(
        self,
        code: int,
        names: Iterable[str],
        timeout: float = REPLY_TIMEOUT_S,
        accept: Callable[[Reply], bool] | None = None,
    ) -> dict[str, Reply]:
        """Each named joint's first reply with `code` (that `accept` takes), until all came or `timeout` ran out.

        A joint missing from the result did not answer in time. Each Reply carries its receive timestamp.
        """
        pending = {j.config.can_id: j for j in self._targets(names)}
        got: dict[str, Reply] = {}

        def wanted(frame: Frame) -> bool:
            joint = pending.get(frame.can_id)
            if joint is None:
                return False
            reply = joint.board.parse(frame)
            return reply is not None and reply.code == code and (accept is None or accept(reply))

        deadline = time.monotonic() + timeout
        while pending:
            frame = self.can.receive(timeout=max(0.0, deadline - time.monotonic()), match=wanted)
            if frame is None:
                break
            joint = pending.pop(frame.can_id)
            reply = joint.board.parse(frame)
            assert reply is not None   # wanted() took only a frame that parses
            got[joint.name] = reply
        return got

    def request_all(
        self,
        data: bytes,
        names: Iterable[str] | None = None,
        timeout: float = REPLY_TIMEOUT_S,
        accept: Callable[[Reply], bool] | None = None,
    ) -> dict[str, Reply]:
        """Drop kept replies with data[0]'s code, send `data` to every board, collect their replies."""
        targets = self._targets(names)
        ids = {j.config.can_id for j in targets}
        self.can.discard(lambda f: f.can_id in ids and len(f.data) > 1 and f.data[0] == data[0])
        sent = self.send_all(data, [j.name for j in targets])
        return self.collect(data[0], sent, timeout, accept)

    def emergency_stop(self, priority: bool = False, timeout: float = ESTOP_TIMEOUT_S) -> dict[str, bool]:
        """0xF7 to every mapped board, all sent before any reply is awaited; returns which confirmed (status 1).

        `priority`: motor_driver.emergency_stop_request (V1.0.9's immediate form; the plain form is the default
        while the boards' firmware is unknown). A send that fails (python-can raising) is reported False and does not
        keep the stop from the boards after it.
        """
        request = emergency_stop_request(priority)
        sent, failed = [], []
        for joint in self.mapped:
            try:
                joint.board.send(request)
                sent.append(joint.name)
            except CanError:
                failed.append(joint.name)
        replies = self.collect(Cmd.EMERGENCY_STOP, sent, timeout)
        result = {n: n in replies and replies[n].status == 1 for n in sent}
        return result | dict.fromkeys(failed, False)

    def read_positions(self, names: Iterable[str] | None = None, timeout: float = REPLY_TIMEOUT_S) -> dict[str, JointSample]:
        """Every board's 0x31 encoder value as a joint angle (Joint.zero_deg), with the reply's timestamp."""
        replies = self.request_all(bytes([Cmd.READ_ENCODER]), names, timeout)
        samples = {}
        for name, reply in replies.items():
            counts = decode_encoder(reply)
            if counts is not None:
                samples[name] = JointSample(self.joints[name].counts_to_deg(counts), counts, reply.timestamp)
        return samples

    def move_joints(
        self,
        targets: Mapping[str, float],
        speed_dps: float | Mapping[str, float],
        acc: int,
        limits: Mapping[str, JointLimits] | None = None,
        timeout: float = REPLY_TIMEOUT_S,
    ) -> dict[str, bool]:
        """Move each named joint to its angle (0xF5), at `speed_dps` (one for all, or per joint) with ramp `acc`.

        Every target is checked against the arm's limits and `limits` (the caller's, per joint) before any frame
        is sent: one bad target raises joint.LimitError and nothing moves. Returns which boards accepted (1 or 2).
        """
        frames = {}
        for name, deg in targets.items():
            joint = self._targets([name])[0]
            speed = speed_dps if isinstance(speed_dps, int | float) else speed_dps[name]
            counts, rpm = joint.check_move(deg, speed, acc, None if limits is None else limits.get(name))
            frames[name] = move_to_axis_request(counts, rpm, acc)
        ids = {self.joints[n].config.can_id for n in frames}
        self.can.discard(lambda f: f.can_id in ids and len(f.data) > 1 and f.data[0] == Cmd.MOVE_TO_AXIS)
        for name, data in frames.items():
            self.joints[name].board.send(data)
        replies = self.collect(Cmd.MOVE_TO_AXIS, frames, timeout)
        return {n: n in replies and replies[n].status in (1, 2) for n in frames}
