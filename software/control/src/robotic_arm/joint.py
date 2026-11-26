"""Layer 3: one joint - joint degrees <-> its board's encoder counts, and the limits a move must keep.

The arm's own hard limits come from config.JOINTS; a caller narrows them per run with JointLimits (an experiment's
numbers stay with the experiment). Every check runs before anything is sent.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import NamedTuple

from robotic_arm.config import ENCODER_COUNTS_PER_REV, JointConfig
from robotic_arm.motor_driver import INT24_MAX, MAX_ACC, MAX_SPEED_RPM, MotorDriver


class LimitError(ValueError):
    """A move outside the arm's limits or the caller's."""


@dataclass(frozen=True)
class JointLimits:
    """A caller's bounds for one joint, on top of the arm's own. The defaults bound nothing.

    max_acc: the steepest MKS ramp allowed (1-255, larger = steeper). acc 0 is no ramp at all - the steepest - so a
    cap below MAX_ACC refuses it too.
    """

    min_deg: float = -math.inf
    max_deg: float = math.inf
    max_speed_dps: float = math.inf
    max_acc: int = MAX_ACC


class JointSample(NamedTuple):
    """A joint's position from its board's 0x31 reply."""

    deg: float
    counts: int
    timestamp: float   # the reply's python-can receive timestamp


class Joint:
    """A joint of the arm and the board that drives it (None when no board is mapped to it).

    zero_deg is the joint angle at the board's encoder zero. It defaults to the config's home_deg, which holds once
    the board has homed (0x91 marks the switch's edge as its zero); before that the zero is wherever the board
    powered on or was last set (0x92), and a caller that knows the joint's angle there sets zero_deg itself.
    """

    def __init__(self, config: JointConfig, motor: MotorDriver | None = None, zero_deg: float | None = None):
        self.config = config
        self.motor = motor
        self.zero_deg = config.home_deg if zero_deg is None else zero_deg

    def __repr__(self) -> str:
        can_id = "unmapped" if self.config.can_id is None else f"{self.config.can_id:#04x}"
        return f"Joint({self.config.name}, {can_id})"

    @property
    def name(self) -> str:
        return self.config.name

    @property
    def board(self) -> MotorDriver:
        """The joint's board - a ValueError for a joint with none mapped (Arm.mapped holds only joints with one)."""
        if self.motor is None:
            raise ValueError(f"no board mapped to {self.name}")
        return self.motor

    @property
    def counts_per_deg(self) -> float:
        """Encoder counts per joint degree: a motor turn is ENCODER_COUNTS_PER_REV, a joint degree 1/gear_ratio of a
        motor degree."""
        return ENCODER_COUNTS_PER_REV / 360.0 / self.config.gear_ratio

    def deg_to_counts(self, deg: float) -> int:
        return round(self.config.sign * (deg - self.zero_deg) * self.counts_per_deg)

    def counts_to_deg(self, counts: int) -> float:
        return self.zero_deg + self.config.sign * counts / self.counts_per_deg

    def dps_to_rpm(self, dps: float) -> float:
        """Joint degrees per second -> motor RPM."""
        return dps / 6.0 / self.config.gear_ratio

    def rpm_to_dps(self, rpm: float) -> float:
        return rpm * 6.0 * self.config.gear_ratio

    def limits(self, caller: JointLimits | None = None) -> JointLimits:
        """The arm's hard limits narrowed by the caller's."""
        lo, hi = self.config.limits_deg
        if caller is None:
            return JointLimits(lo, hi)
        return JointLimits(max(lo, caller.min_deg), min(hi, caller.max_deg), caller.max_speed_dps, caller.max_acc)

    def check_move(self, deg: float, speed_dps: float, acc: int, caller: JointLimits | None = None) -> tuple[int, int]:
        """Check a move to `deg` at `speed_dps` with ramp `acc`; returns the board's (position counts, speed RPM).

        Raises LimitError when the move breaks a limit, ValueError when an argument cannot be a move at all.
        """
        lim = self.limits(caller)
        if not math.isfinite(deg) or not lim.min_deg <= deg <= lim.max_deg:
            raise LimitError(f"{self.name}: {deg} deg outside {lim.min_deg}..{lim.max_deg}")
        if not speed_dps > 0:
            raise ValueError(f"{self.name}: speed must be > 0 deg/s, got {speed_dps}")
        if speed_dps > lim.max_speed_dps:
            raise LimitError(f"{self.name}: {speed_dps} deg/s over the {lim.max_speed_dps} deg/s limit")
        if not 0 <= acc <= MAX_ACC:
            raise ValueError(f"{self.name}: acc {acc} outside 0..{MAX_ACC}")
        if acc > lim.max_acc or (acc == 0 and lim.max_acc < MAX_ACC):
            raise LimitError(f"{self.name}: acc {acc} steeper than the limit {lim.max_acc} (0 = no ramp)")
        rpm = max(1, round(self.dps_to_rpm(speed_dps)))
        if rpm > MAX_SPEED_RPM:
            raise LimitError(f"{self.name}: {speed_dps} deg/s needs {rpm} RPM, over the board's {MAX_SPEED_RPM}")
        counts = self.deg_to_counts(deg)
        if abs(counts) > INT24_MAX:
            raise LimitError(f"{self.name}: {deg} deg is {counts} counts, past the board's +/-{INT24_MAX}")
        return counts, rpm
