"""Several motors on one CAN bus: the dashboard's Rig.

One BusWorker polls every board, and each motor is a Bench sharing it - its own joint, units, limits, profile,
reference and tests. One motor is active: the page's panels drive and graph it, and it is polled fastest (the others
every BACKGROUND_FAST_S). STOP stops them all, ahead of everything queued. A move together checks every motor's
target first - one refused and nothing moves - then sends the F5s back to back, each motor's speed scaled so they all
arrive at the same time; if a board does not take its move, every motor is stopped.
"""

from __future__ import annotations

import threading
import time
from typing import Any

from robotic_arm import config
from robotic_arm.can_interface import CANInterface
from robotic_arm.dashboard.bench import (
    DEFAULT_JOINT_SPEED_CAP,
    REPLY_S,
    Bench,
    Notices,
    Refused,
)
from robotic_arm.dashboard.bus import BusWorker
from robotic_arm.motor_driver import Cmd, move_to_axis_request


class Rig:
    """The motors on the page, listed by CAN id (`ordered`); `active` is the one the panels drive."""

    def __init__(self, can: CANInterface, *, max_joint_speed: float = DEFAULT_JOINT_SPEED_CAP,
                 priority_estop: bool = False, label: str = ""):
        self.worker = BusWorker(can)
        self.notices = Notices()
        self.max_joint_speed, self.priority_estop, self.label = max_joint_speed, priority_estop, label
        self.motors: dict[str, Bench] = {}
        self.active: str | None = None
        self._next = 1
        self._lock = threading.RLock()
        self.worker.start()

    def close(self) -> None:
        for bench in list(self.motors.values()):
            bench.close()
        self.worker.close()

    # --- the motors ---------------------------------------------------------------------------------------------------

    def bench(self, key: str | None = None) -> Bench:
        """The motor `key` (default: the active one)."""
        with self._lock:
            key = key or self.active
            if key not in self.motors:
                raise Refused(f"no motor {key!r} on the page")
            return self.motors[key]

    def ordered(self) -> list[str]:
        """The motors' keys by CAN id: the page's cards and its keys 1-9 go in this order. Sorted on every call, as
        a board's CAN id can change while it is on the page (0x8B)."""
        with self._lock:
            return sorted(self.motors, key=lambda k: self.motors[k].can_id)

    def add_motor(self, joint: str, can_id: int) -> str:
        """Put `joint`'s board at `can_id` on the page; the first one becomes the active one."""
        if not 1 <= can_id <= 0x7FF:
            raise Refused("a CAN id is 1..2047")
        with self._lock:
            if any(b.can_id == can_id for b in self.motors.values()):
                raise Refused(f"a motor at CAN id {can_id} is on the page already")
            key = f"m{self._next}"
            self._next += 1
            bench = Bench(worker=self.worker, notices=self.notices, joint=joint, can_id=can_id,
                          max_joint_speed=self.max_joint_speed, priority_estop=self.priority_estop, label=self.label)
            bench.tag = f"{joint}: "
            self.motors[key] = bench
            if self.active is None:
                self.active = key
            else:
                bench.set_background(True)
        self.notices.add(f"Added {joint} at CAN id {can_id}")
        return key

    def remove_motor(self, key: str) -> None:
        with self._lock:
            bench = self.bench(key)
            if len(self.motors) == 1:
                raise Refused("the last motor stays on the page")
            bench._require_idle("taking it off the page")
            bench.close()
            del self.motors[key]
            if self.active == key:
                self.active = self.ordered()[0]
                self.motors[self.active].set_background(False)

    def set_active(self, key: str) -> None:
        with self._lock:
            new = self.bench(key)
            if key == self.active:
                return
            if self.active in self.motors:
                self.motors[self.active].set_background(True)
            new.set_background(False)
            self.active = key

    def discover(self, first: int = 1, last: int = 16) -> list[str]:
        """Scan `first`..`last` (0x40) and add every answering board whose CAN id is a joint's in config.JOINTS."""
        found = self.bench().scan(first, last) if self.motors else self._scan_without_motor(first, last)
        added = []
        with self._lock:
            have = {b.can_id for b in self.motors.values()}
        for board in found:
            can_id = board["can_id"]
            if can_id in have:
                continue
            joint = next((j.name for j in config.JOINTS if j.can_id == can_id), None)
            if joint is None:
                self.notices.add(f"A board answers at CAN id {can_id}, which no joint in config.JOINTS has: add it with "
                                 "+ and pick its joint", "warn")
                continue
            added.append(self.add_motor(joint, can_id))
        return added

    def _scan_without_motor(self, first: int, last: int) -> list[dict[str, Any]]:
        futures = {i: self.worker.submit(i, bytes([Cmd.READ_VERSION]), timeout=0.4) for i in range(first, last + 1)}
        return [{"can_id": i} for i, f in futures.items() if f.result(3.0) is not None]

    def touch(self) -> None:
        for bench in list(self.motors.values()):
            bench.touch()

    # --- every motor at once ------------------------------------------------------------------------------------------

    def halt_all(self, reason: str = "STOP pressed") -> dict[str, bool | None]:
        """STOP every motor: all the stops go out first, then the confirmations are collected (key -> F7 confirmed,
        None when the board did not answer)."""
        seq = self.worker.event_seq
        with self._lock:
            motors = dict(self.motors)
        for bench in motors.values():
            with bench._lock:
                bench._halt_frames(reason)
        by_id = {b.can_id: key for key, b in motors.items()}
        got: dict[str, bool] = {}
        deadline = time.monotonic() + 0.5
        while len(got) < len(by_id) and time.monotonic() < deadline:
            reply = self.worker.wait_event(
                seq, lambda r: r.code == Cmd.EMERGENCY_STOP and r.can_id in by_id and by_id[r.can_id] not in got,
                deadline - time.monotonic())
            if reply is None:
                break
            got[by_id[reply.can_id]] = reply.status == 1
        return {key: got.get(key) for key in motors}

    def move_together(self, targets: dict[str, float], speed_pct: float, acc: int, *, together: bool = True) -> dict[str, Any]:
        """Move each motor `key` to its target (its mode's unit) at `speed_pct` % of its speed cap - scaled down so they
        all arrive together, with `together`. Every move is checked before any is sent."""
        if not targets:
            raise Refused("no targets")
        if not 0 < speed_pct <= 100:
            raise Refused("speed: 1..100 % of each motor's cap")
        plans = []
        for key, value in targets.items():
            bench = self.bench(key)
            with bench._lock:
                if bench.routine is not None and bench.routine.active:
                    raise Refused(f"{bench.joint_config.name}: a test is running")
                if bench.motion in ("home", "calibrate"):
                    raise Refused(f"{bench.joint_config.name} is {bench.motion}")
                bench._require_zeroed()
                here = bench.value_now()
                if here is None or bench._stale("counts", 0.5):
                    raise Refused(f"{bench.joint_config.name}: no fresh encoder reading")
                plans.append((key, bench, float(value), abs(float(value) - here), bench.speed_cap() * speed_pct / 100))
        moving = [p for p in plans if p[3] > 1e-9]   # (key, bench, target, distance, speed)
        if not moving:
            return {"message": "every motor is at its target already"}
        if together:
            duration = max(dist / speed for _, _, _, dist, speed in moving)
            moving = [(key, bench, value, dist, max(dist / duration, 1e-6)) for key, bench, value, dist, _ in moving]
        checked = []
        for key, bench, value, _, speed in moving:
            try:
                counts, rpm = bench.check(value, speed, acc)
            except Refused as e:
                raise Refused(f"{bench.joint_config.name}: {e} - nothing moved") from e
            checked.append((key, bench, counts, rpm))
        seq = self.worker.event_seq
        frames = []
        for key, bench, counts, rpm in checked:
            with bench._lock:
                bench._stop_shaper()
                shaped = bench.profile.kind == "scurve"
                ramp = bench.profile.follow_acc if shaped else acc
                bench.motion, bench._motion_since, bench._last_acc = "move", time.monotonic(), ramp
                bench._window_stop = False
                start = bench.readout.counts
            frames.append((key, bench, counts, rpm, shaped, start,
                           move_to_axis_request(counts, max(1, round(0.03 * rpm)) if shaped else rpm, ramp)))
        for _, bench, _, _, _, _, data in frames:   # back to back: the worker sends them in one go
            self.worker.send(bench.can_id, data)
        pending = {bench.can_id: key for key, bench, *_ in frames}
        started: dict[str, bool] = {}
        deadline = time.monotonic() + REPLY_S * 2
        while len(started) < len(pending) and time.monotonic() < deadline:
            reply = self.worker.wait_event(
                seq, lambda r: r.code == Cmd.MOVE_TO_AXIS and r.can_id in pending and pending[r.can_id] not in started,
                deadline - time.monotonic())
            if reply is None:
                break
            started[pending[reply.can_id]] = reply.status in (1, 2)
        failed = [self.motors[k].joint_config.name for k in pending.values() if not started.get(k)]
        if failed:
            for _, bench, *_ in frames:
                bench.stop()
            raise Refused(f"{', '.join(failed)} did not take the move (enabled? work mode SR_vFOC?): every motor stopped")
        for _, bench, counts, rpm, shaped, start, _ in frames:
            if shaped and start is not None:
                bench._start_shaper(start, rpm, target=counts)
        return {"message": f"{len(frames)} motors moving together"}

    # --- the page's view ------------------------------------------------------------------------------------------------

    def state(self, samples_after: int = 0, log_after: int = 0, notices_after: int = 0,
              active: str | None = None) -> dict[str, Any]:
        """The active motor's full state, plus a card for every motor. A page that still shows another motor
        (`active` differs) gets the active one's samples from the start."""
        with self._lock:
            key = self.active
            if key is None:
                raise Refused("no motor on the page")
            if active != key:
                samples_after = 0
            motors = [{"key": k, "active": k == key, **self.motors[k].card()} for k in self.ordered()]
            bench = self.motors[key]
        return {**bench.state(samples_after, log_after, notices_after), "active": key, "motors": motors}
