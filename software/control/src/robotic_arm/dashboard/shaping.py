"""Shaped moves: the dashboard shapes a move's speed in time and streams it to the board as F5 updates.

The board itself only ramps in a straight line (9.1: acc, 1 RPM every (256 - acc) x 50 us). A Shaper sends F5 every
1 / rate_hz s - the same target, a new speed - so the speed follows `curve` (0 -> 1 over the ease time) on the way up
and, on the way down, the same curve by the distance left: re-measured from the encoder at every update, so the stop
lands on the target whatever the board's lag. The board follows each update with the steep `follow_acc` ramp. F5 takes
a new speed and target while it runs since V1.0.5 (11.4 note 2). A jog's target rolls ahead of the motor (never past a
limit); released, it eases down by time and ends with the board's own ramped stop.

Every curve rises from 0 to 1 and covers half the area of the unit square, so a phase of ease time T at top speed v
runs v * T / 2: a stop from speed v takes T * v / v_max seconds and v^2 * T / (2 * v_max) of travel.
"""

from __future__ import annotations

import bisect
import functools
import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING

from robotic_arm.config import ENCODER_COUNTS_PER_REV
from robotic_arm.motor_driver import INT24_MAX, move_to_axis_request

if TYPE_CHECKING:
    from robotic_arm.dashboard.bench import Bench

COUNTS_PER_RPM_S = ENCODER_COUNTS_PER_REV / 60
CURVES: dict[str, Callable[[float], float]] = {
    "linear": lambda x: x,                                  # a straight ramp: the board's own shape, at our pace
    "smoothstep": lambda x: x * x * (3 - 2 * x),            # acceleration starts and ends at 0
    "sine": lambda x: (1 - math.cos(math.pi * x)) / 2,      # likewise, a half cosine
    "quintic": lambda x: x ** 3 * (x * (6 * x - 15) + 10),  # jerk starts and ends at 0 too: the softest
}
TABLE_N = 400


@dataclass
class Profile:
    """How moves are shaped. kind 'board': the board's own straight ramp (the acc each move carries). kind 'scurve':
    the dashboard streams the speed along `curve`, reaching full speed in `ease_s`, `rate_hz` updates a second, the
    board following each with ramp `follow_acc`."""

    kind: str = "board"
    curve: str = "smoothstep"
    ease_s: float = 0.6
    rate_hz: float = 50.0
    follow_acc: int = 240


@functools.cache
def _table(curve: str) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """(F(u), f(u)) for u = 0 .. 1 in TABLE_N steps: the curve f and its area F (F(1) = 0.5)."""
    f = CURVES[curve]
    fs = [f(i / TABLE_N) for i in range(TABLE_N + 1)]
    area = [0.0]
    for i in range(1, TABLE_N + 1):
        area.append(area[-1] + (fs[i] + fs[i - 1]) / 2 / TABLE_N)
    return tuple(area), tuple(fs)


def speed_by_area(curve: str, area: float) -> float:
    """f(u) at the u where F(u) = `area`: the curve's speed with that much area still to cover."""
    areas, fs = _table(curve)
    a = min(max(area, 0.0), areas[-1])
    i = bisect.bisect_left(areas, a)
    if i <= 0:
        return fs[0]
    w = (a - areas[i - 1]) / (areas[i] - areas[i - 1]) if areas[i] > areas[i - 1] else 0.0
    return fs[i - 1] + w * (fs[i] - fs[i - 1])


def phase_at_speed(curve: str, fraction: float) -> float:
    """The u where f(u) = `fraction` (the curves rise monotonically)."""
    _, fs = _table(curve)
    i = bisect.bisect_left(fs, min(max(fraction, 0.0), 1.0))
    return min(i, TABLE_N) / TABLE_N


class Shaper(threading.Thread):
    """Streams one shaped move (to `target`, counts) or jog (`jog_dir` +1 / -1 in counts) at up to `vmax` RPM.

    It runs while the bench's shaping generation is `gen`: Bench._stop_shaper() bumps it, and every send is made under
    the bench's lock after checking it, so nothing is sent after a stop.
    """

    def __init__(self, bench: Bench, gen: int, start: int, vmax: float, *, target: int | None = None,
                 jog_dir: int | None = None, lead_s: float = 2.0):
        super().__init__(name="dashboard-shaper", daemon=True)
        p = bench.profile
        self.bench, self.gen, self.vmax = bench, gen, float(vmax)
        self.curve, self.ease, self.period, self.follow = p.curve, p.ease_s, 1.0 / p.rate_hz, p.follow_acc
        self.f = CURVES[p.curve]
        self.target, self.jog_dir, self.lead_s = target, jog_dir, lead_s
        self.dir = jog_dir if jog_dir is not None else (1 if (target or 0) >= start else -1)
        self.t0 = time.monotonic()
        self.release_at: float | None = None
        self.v_release = 0.0
        self.decel: tuple[float, float] | None = None   # (speed at the decel's start, its length in s)
        self.v = 0.0
        self.last_sent: tuple[int, int] | None = None
        self.done = threading.Event()

    @property
    def floor(self) -> int:
        """The lowest RPM sent: below it the board's own ramp finishes the move on its target."""
        return max(1, round(0.03 * self.vmax))

    # called under the bench's lock
    def release(self) -> None:
        """A jog let go: ease down by time from the speed it has."""
        if self.release_at is None:
            self.release_at, self.v_release = time.monotonic(), max(self.v, 1e-6)

    def resume(self) -> None:
        """A jog pressed again while easing down: ease back up from the speed it has, without a jolt."""
        if self.release_at is not None:
            self.release_at = None
            self.t0 = time.monotonic() - self.ease * phase_at_speed(self.curve, self.v / self.vmax)

    def run(self) -> None:
        try:
            while self._step():
                time.sleep(self.period)
        finally:
            self.done.set()
            self.bench._shaper_done(self)

    def _step(self) -> bool:
        b = self.bench
        with b._lock:
            if b._shape_gen != self.gen:
                return False
            counts = b.readout.counts
            if counts is None:
                return True
            now = time.monotonic()
            d = self.dir
            if self.jog_dir is None:
                target, bounded = self.target, True
            else:
                target = counts + d * max(self.vmax * COUNTS_PER_RPM_S * self.lead_s, 64)
                lo, hi, _, _ = b._bounds()
                bounded = (d > 0 and hi is not None and target >= hi) or (d < 0 and lo is not None and target <= lo)
                if bounded:
                    target = hi if d > 0 else lo
                target = max(-INT24_MAX, min(INT24_MAX, round(target)))
            rem = (target - counts) * d
            if self.jog_dir is None and rem <= 2:
                return False                                   # there: the last F5 lands it
            if self.release_at is None:
                v_time = self.vmax * self.f(min(1.0, (now - self.t0) / self.ease))
            else:
                t_rel = self.ease * self.v_release / self.vmax
                tau = now - self.release_at
                if tau >= t_rel:
                    b._last_acc = self.follow
                    b.worker.send(b.can_id, move_to_axis_request(0, 0, self.follow))   # the board's own last stop
                    b.readout.cmd_rpm = 0.0
                    return False
                v_time = self.v_release * self.f(1 - tau / t_rel)
            v_dist = math.inf
            if bounded:
                if self.decel is None:
                    t_stop = self.ease * self.v / self.vmax
                    if rem <= self.v * COUNTS_PER_RPM_S * t_stop / 2 * 1.02 + 2:
                        self.decel = (self.v, t_stop)
                if self.decel is not None:
                    vp, tp = self.decel
                    v_dist = vp * speed_by_area(self.curve, rem / (vp * COUNTS_PER_RPM_S * tp)) if vp > 0 and tp > 0 else 0.0
            self.v = max(0.0, min(v_time, v_dist, self.vmax))
            rpm = max(self.floor, min(3000, round(self.v)))
            b.readout.cmd_rpm = self.v * d
            if (target, rpm) != self.last_sent:
                b._last_acc = self.follow
                b.worker.send(b.can_id, move_to_axis_request(target, rpm, self.follow))
                self.last_sent = (target, rpm)
        return True
