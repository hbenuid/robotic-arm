"""The dashboard's model of the board under test: its joint, the units and limits of each mode, the safety rules.

Two modes. **Joint mode**: joint degrees through the joint's gear ratio and sign (config.JOINTS), every move checked
by Joint.check_move against the arm's hard limits narrowed by the page's, at no more than `max_joint_speed` deg/s.
It needs the joint's reference - the joint angle at the encoder's zero - from homing or from the user saying where the
joint is; until then nothing moves in joint mode but the direction check's small confirmed step. **Bench mode**:
degrees and RPM of the motor's own shaft, inside a travel window round the encoder's zero, for a motor off the arm.

The safety rules, all on the bus thread's tick so they hold whatever the page does:
- STOP (halt) sends F7 and an immediate F5 stop (and an F6 stop after a speed run) ahead of everything queued.
- A jog moves toward the window's edge and stops (ramped) when the page's keep-alives stop for JOG_KEEPALIVE_S.
- Any motion or test stops when the page has not been heard from for PAGE_TIMEOUT_S.
- A bench speed run stops when it leaves the window.
- A board that goes silent for longer than OFFLINE_S and comes back loses joint mode's reference: if it lost power,
  its encoder's zero moved.
"""

from __future__ import annotations

import collections
import csv
import io
import math
import threading
import time
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import asdict, dataclass, field, replace
from typing import Any

from robotic_arm import config
from robotic_arm.can_interface import CANInterface
from robotic_arm.config import ENCODER_COUNTS_PER_REV, JointConfig
from robotic_arm.dashboard.bus import BusWorker
from robotic_arm.joint import Joint, JointLimits, LimitError
from robotic_arm.motor_driver import (
    HARDWARE_NAMES,
    INT24_MAX,
    MAX_ACC,
    STATUS_NAMES,
    Checksum,
    Cmd,
    Reply,
    decode_encoder,
    decode_int16,
    decode_int32,
    decode_power,
    decode_version,
    emergency_stop_request,
    encode_uint16,
    move_relative_axis_request,
    move_to_axis_request,
    speed_request,
)

JOG_KEEPALIVE_S = 0.4     # a jog stops this long after the page's last keep-alive (it sends one every 0.1 s)
PAGE_TIMEOUT_S = 2.0      # motion and tests stop when the page is silent this long (it pings every 0.25 s)
OFFLINE_S = 1.0           # no reply for this long: the board is shown offline
REPLY_S = 0.15            # the dashboard's commands wait this long for their reply
BENCH_RPM_CEILING = 1000  # the manual: no immediate stop (F7, acc 0) above 1000 RPM
DEFAULT_JOINT_SPEED_CAP = 10.0
MOTOR_DEG_PER_COUNT = 360.0 / ENCODER_COUNTS_PER_REV
FAST_POLL = (Cmd.READ_ENCODER, Cmd.READ_SPEED, Cmd.READ_STATUS)
SLOW_POLL = (Cmd.READ_ENABLED, Cmd.READ_STALL, Cmd.READ_IO, Cmd.READ_POWER, Cmd.READ_ANGLE_ERROR,
             Cmd.READ_HOME_STATUS)
SAMPLES_MAX = 120_000     # about 40 min of positions at the 20 ms poll
RECORD_MAX = 2_000_000
# Codes the raw box refuses: they move the motor past every check here. The motion panel and the tests send them.
RAW_REFUSED = {0x80: "calibration", 0x91: "homing", 0x9A: "single-turn zero return", 0xF4: "relative move",
               0xF5: "absolute move", 0xF6: "speed run", 0xFD: "pulse move", 0xFE: "pulse move",
               0x4A: "synchronized move", 0x4B: "synchronized start"}
# Settings the Settings panel writes: code -> (name, bytes, lowest, highest).
SETTINGS = {
    0x82: ("work mode", 1, 0, 5),
    0x83: ("working current, mA", 2, 0, 3000),
    0x84: ("microsteps", 1, 1, 255),
    0x88: ("stall protection", 1, 0, 1),
    0x8B: ("CAN id", 2, 1, 0x7FF),
    0x98: ("heartbeat, ms (0 = off)", 4, 0, 0xFFFF_FFFF),
}
ACTIONS = {0x3D: "release stall", 0x41: "restart", 0x3F: "restore factory defaults"}
READABLE = (0x82, 0x83, 0x84, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x9E)   # 0x00 read-backs the Settings panel shows
WORK_MODES = {0: "CR_OPEN", 1: "CR_CLOSE", 2: "CR_vFOC", 3: "SR_OPEN", 4: "SR_CLOSE", 5: "SR_vFOC"}


class Refused(Exception):
    """A command the dashboard did not send, or one the board refused; the message says why."""


@dataclass
class Limits:
    """The page's limits. Joint mode: joint degrees and deg/s, inside the arm's own. Bench mode: motor degrees round
    the encoder's zero and RPM."""

    joint_min: float
    joint_max: float
    joint_speed: float
    bench_min: float = -360.0
    bench_max: float = 360.0
    bench_rpm: float = 60.0


@dataclass
class Readout:
    """The board's latest values and when each came (time.monotonic()); None = not read yet."""

    counts: int | None = None
    speed_rpm: int | None = None
    status: int | None = None
    enabled: bool | None = None
    stalled: bool | None = None
    io: int | None = None
    millivolts: int | None = None
    milliamps: int | None = None
    angle_error: int | None = None
    home_status: tuple[int, ...] | None = None
    version: str | None = None
    updated: dict[str, float] = field(default_factory=dict)


SAMPLE_FIELDS = ("t", "counts", "speed_rpm", "status", "milliamps", "angle_error", "io")


class Bench:
    """The board under test and the dashboard's rules for it. Thread-safe: the page's requests, the test routines
    and the bus thread all call in."""

    def __init__(self, can: CANInterface, *, joint: str = config.JOINTS[0].name, can_id: int | None = None,
                 max_joint_speed: float = DEFAULT_JOINT_SPEED_CAP, priority_estop: bool = False,
                 checksum: Checksum = Checksum.SUM8, label: str = ""):
        if not max_joint_speed > 0:
            raise ValueError("max_joint_speed must be > 0")
        self.label = label
        self.max_joint_speed = float(max_joint_speed)
        self.priority_estop = priority_estop
        self.worker = BusWorker(can, checksum)
        self._lock = threading.RLock()
        self.mode = "joint"
        self.joint_config: JointConfig = self._config_named(joint)
        self.can_id = can_id or self.joint_config.can_id or 1
        self.sign = self.joint_config.sign
        self.zero_deg: float | None = None
        self.reference = "not set"
        lo, hi = self.joint_config.limits_deg
        self.limits = Limits(lo, hi, self.max_joint_speed)
        self.readout = Readout()
        self.samples: collections.deque[tuple] = collections.deque(maxlen=SAMPLES_MAX)
        self.sample_seq = 0
        self._recording: list[tuple] | None = None
        self.notices: collections.deque[dict[str, Any]] = collections.deque(maxlen=200)
        self._notice_seq = 0
        self.findings: list[str] = []
        self.motion: str | None = None          # "move", "jog", "run", "home", "calibrate", "step", None
        self._motion_since = 0.0
        self._last_acc = config.DEFAULT_ACC
        self._jog: int | None = None
        self._jog_keepalive = 0.0
        self._window_stop = False
        self._last_page = time.monotonic()
        self._last_page_halt = 0.0
        self._online = False
        self._offline_since: float | None = None
        self._version_asked = 0.0
        self._last_recording: list[tuple] = []
        self.routine = None                     # a routines.Routine while one runs or after it ends
        self._poll_profile = (FAST_POLL, SLOW_POLL, 0.02)
        self.worker.on_reply = self._on_reply
        self.worker.on_tick = self._tick
        self.worker.driver(self.can_id)
        self.worker.set_poll(self.can_id, *self._poll_profile)
        self.worker.start()

    def close(self) -> None:
        if self.routine is not None:
            self.routine.cancel.set()
        self.worker.close()

    # --- units ------------------------------------------------------------------------------------------------------

    @staticmethod
    def _config_named(name: str) -> JointConfig:
        for c in config.JOINTS:
            if c.name == name:
                return c
        raise Refused(f"no joint named {name!r} in config.JOINTS")

    def joint(self) -> Joint:
        """The joint with the session's sign and reference (zero_deg 0 when there is none)."""
        cfg = replace(self.joint_config, can_id=self.can_id, sign=self.sign)
        return Joint(cfg, None, 0.0 if self.zero_deg is None else self.zero_deg)

    def counts_per_unit(self) -> float:
        """Encoder counts per the mode's position unit (a joint degree, or a motor degree)."""
        return self.joint().counts_per_deg if self.mode == "joint" else 1 / MOTOR_DEG_PER_COUNT

    def to_value(self, counts: int) -> float | None:
        """`counts` in the mode's unit; None in joint mode without a reference."""
        with self._lock:
            if self.mode == "bench":
                return counts * MOTOR_DEG_PER_COUNT
            if self.zero_deg is None:
                return None
            return self.joint().counts_to_deg(counts)

    def value_now(self) -> float | None:
        counts = self.readout.counts
        return None if counts is None else self.to_value(counts)

    def window(self) -> tuple[float, float]:
        """The travel the mode allows: joint mode the page's limits inside the arm's, bench mode the window."""
        with self._lock:
            if self.mode == "bench":
                return self.limits.bench_min, self.limits.bench_max
            lim = self.joint().limits(JointLimits(self.limits.joint_min, self.limits.joint_max))
            return lim.min_deg, lim.max_deg

    def speed_cap(self) -> float:
        """The fastest the mode allows: deg/s of the joint, or RPM of the motor."""
        with self._lock:
            if self.mode == "bench":
                return min(self.limits.bench_rpm, BENCH_RPM_CEILING)
            return min(self.limits.joint_speed, self.max_joint_speed)

    def speed_to_rpm(self, speed: float) -> int:
        """`speed` in the mode's unit -> whole motor RPM, never above the cap (rounded down where rounding would pass it)."""
        cap = self.speed_cap()
        if not 0 < speed <= cap + 1e-9:
            raise Refused(f"speed {speed:g} outside 0..{cap:g} {'RPM' if self.mode == 'bench' else 'deg/s'} (the cap)")
        if self.mode == "bench":
            return max(1, min(math.floor(cap + 1e-9), round(speed)))
        joint = self.joint()
        rpm = max(1, round(joint.dps_to_rpm(speed)))
        while rpm > 1 and joint.rpm_to_dps(rpm) > cap + 1e-9:
            rpm -= 1
        if joint.rpm_to_dps(rpm) > cap + 1e-9:
            raise Refused(f"{cap:g} deg/s is under the board's 1 RPM ({joint.rpm_to_dps(1):.2f} deg/s at {joint.name})")
        return rpm

    def check(self, value: float, speed: float, acc: int) -> tuple[int, int]:
        """(position counts, RPM) for a move to `value` at `speed` (the mode's units); raises Refused."""
        with self._lock:
            if not 0 <= acc <= MAX_ACC:
                raise Refused(f"acc {acc} outside 0..{MAX_ACC}")
            rpm = self.speed_to_rpm(speed)
            if self.mode == "bench":
                lo, hi = self.window()
                if not math.isfinite(value) or not lo <= value <= hi:
                    raise Refused(f"{value:g} deg of the motor is outside the bench window {lo:g}..{hi:g}")
                counts = round(value / MOTOR_DEG_PER_COUNT)
                if abs(counts) > INT24_MAX:
                    raise Refused(f"{value:g} deg is past the board's +/-{INT24_MAX} counts")
                return counts, rpm
            if self.zero_deg is None:
                raise Refused("joint mode needs the joint's reference first: home it, or set where it is now "
                              "(Limits & reference)")
            lim = JointLimits(self.limits.joint_min, self.limits.joint_max, math.inf)
            try:
                counts, _ = self.joint().check_move(value, speed, acc, lim)
            except (LimitError, ValueError) as e:
                raise Refused(str(e)) from e
            return counts, rpm

    # --- page settings ----------------------------------------------------------------------------------------------

    def touch(self) -> None:
        """The page is there (every request it makes)."""
        self._last_page = time.monotonic()

    def notice(self, text: str, level: str = "info") -> None:
        with self._lock:
            self._notice_seq += 1
            self.notices.append({"seq": self._notice_seq, "t": time.time(), "level": level, "text": text})
        self.worker.note(text)

    def _require_idle(self, what: str) -> None:
        if self.routine is not None and self.routine.active:
            raise Refused(f"a test is running ({self.routine.title}): cancel it before {what}")
        if self.motion is not None:
            raise Refused(f"the motor is busy ({self.motion}): stop it before {what}")

    def select(self, joint: str, can_id: int) -> None:
        """Drive `joint`'s board at `can_id` (the joint's own id need not be set on the board yet)."""
        if not 1 <= can_id <= 0x7FF:
            raise Refused("a CAN id is 1..2047")
        cfg = self._config_named(joint)
        with self._lock:
            self._require_idle("changing the board")
            changed = (cfg.name, can_id) != (self.joint_config.name, self.can_id)
            self.joint_config, self.can_id, self.sign = cfg, can_id, cfg.sign
            if changed:
                self.zero_deg, self.reference = None, "not set"
                self.readout = Readout()
            lo, hi = cfg.limits_deg
            self.limits.joint_min, self.limits.joint_max = lo, hi
            self.worker.driver(can_id)
            self.worker.set_poll(can_id, *self._poll_profile)
        self.notice(f"Board: {cfg.name} at CAN id {can_id} (0x{can_id:02X})")

    def set_mode(self, mode: str) -> None:
        if mode not in ("joint", "bench"):
            raise Refused("the mode is joint or bench")
        with self._lock:
            if mode != self.mode:
                self._require_idle("changing the mode")
                self.mode = mode
        if mode == "bench":
            self.notice("Bench mode: motor degrees and RPM inside the bench window; the arm's limits are not "
                        "checked - use it with the motor off the arm", "warn")

    def set_limits(self, **values: float) -> None:
        with self._lock:
            new = replace(self.limits, **{k: float(v) for k, v in values.items() if k in asdict(self.limits)})
            lo, hi = self.joint_config.limits_deg
            if not lo <= new.joint_min < new.joint_max <= hi:
                raise Refused(f"joint limits must lie inside the arm's {lo:g}..{hi:g} deg, min below max")
            if not 0 < new.joint_speed <= self.max_joint_speed:
                raise Refused(f"joint speed limit 0..{self.max_joint_speed:g} deg/s (the dashboard's cap; "
                              f"--max-joint-speed raises it)")
            if not new.bench_min < new.bench_max or abs(new.bench_min) > 360 * 500 or abs(new.bench_max) > 360 * 500:
                raise Refused("bench window: min below max, within +/-500 motor turns")
            if not 1 <= new.bench_rpm <= BENCH_RPM_CEILING:
                raise Refused(f"bench speed limit 1..{BENCH_RPM_CEILING} RPM")
            self.limits = new

    def set_sign(self, sign: int, *, routine: bool = False) -> None:
        if sign not in (1, -1):
            raise Refused("sign is +1 or -1")
        with self._lock:
            if not routine:
                self._require_idle("changing the sign")
            if sign != self.sign:
                self.sign = sign
                if self.zero_deg is not None:
                    self.zero_deg, self.reference = None, "not set"
                    self.notice("Sign changed: the reference is cleared - set it again", "warn")

    def set_reference(self, deg: float) -> None:
        """The joint is at `deg` now: zero_deg follows from the encoder's count."""
        with self._lock:
            self._require_idle("setting the reference")
            if self._moving():
                raise Refused("the motor is moving: set the reference with it stopped")
            counts = self.readout.counts
            if counts is None or self._stale("counts", 0.5):
                raise Refused("no fresh encoder reading: is the board answering?")
            lo, hi = self.joint_config.limits_deg
            if not lo - 30 <= deg <= hi + 30:
                raise Refused(f"{deg:g} deg is far outside {self.joint_config.name}'s range {lo:g}..{hi:g}")
            joint = self.joint()
            self.zero_deg = deg - self.sign * counts / joint.counts_per_deg
            self.reference = f"set by hand: {deg:g} deg at {counts} counts"
        self.notice(f"Reference set: {self.joint_config.name} is at {deg:g} deg now")

    def clear_reference(self) -> None:
        with self._lock:
            self.zero_deg, self.reference = None, "not set"

    def _homed(self) -> None:
        """Homing succeeded: the encoder's zero is the switch's edge, the joint's home_deg."""
        with self._lock:
            self.zero_deg = self.joint_config.home_deg
            self.reference = f"homed: the switch's edge is home_deg {self.joint_config.home_deg:g} deg"

    # --- bus replies and the safety tick (bus thread) -----------------------------------------------------------------

    def _stale(self, key: str, age: float) -> bool:
        t = self.readout.updated.get(key)
        return t is None or time.monotonic() - t > age

    def _on_reply(self, reply: Reply, is_poll: bool) -> None:
        if reply.can_id != self.can_id:
            return
        r, d, code, now = self.readout, reply.data, reply.code, time.monotonic()
        with self._lock:
            if code == Cmd.READ_ENCODER and (counts := decode_encoder(reply)) is not None:
                r.counts, r.updated["counts"] = counts, now
                self._add_sample()
            elif code == Cmd.READ_SPEED and len(d) >= 3:
                r.speed_rpm, r.updated["speed_rpm"] = decode_int16(d[1:3]), now
            elif code == Cmd.READ_STATUS and len(d) >= 2:
                r.status, r.updated["status"] = d[1], now
            elif code == Cmd.READ_ENABLED and len(d) >= 2:
                r.enabled, r.updated["enabled"] = d[1] == 1, now
            elif code == Cmd.READ_STALL and len(d) >= 2:
                r.stalled, r.updated["stalled"] = d[1] == 1, now
            elif code == Cmd.READ_IO and len(d) >= 2:
                r.io, r.updated["io"] = d[1], now
            elif code == Cmd.READ_POWER and (p := decode_power(reply)) is not None:
                r.millivolts, r.milliamps, r.updated["power"] = p.millivolts, p.milliamps, now
            elif code == Cmd.READ_ANGLE_ERROR and len(d) >= 5:
                r.angle_error, r.updated["angle_error"] = decode_int32(d[1:5]), now
            elif code == Cmd.READ_HOME_STATUS and len(d) >= 2:
                r.home_status, r.updated["home_status"] = tuple(d[1:]), now
            elif code == Cmd.READ_VERSION and (v := decode_version(reply)) is not None:
                r.version = f"{v.firmware_text} ({HARDWARE_NAMES.get(v.hardware, v.hardware)})"
            elif (code in (Cmd.MOVE_TO_AXIS, Cmd.MOVE_RELATIVE_AXIS, Cmd.SPEED) and not is_poll
                  and reply.status in (2, 3) and self.motion in ("move", "jog", "run", "step")):
                if reply.status == 3:
                    self.notice("The board stopped the move at a limit (status 3)", "warn")
                self._end_motion()

    def _add_sample(self) -> None:
        r = self.readout
        self.sample_seq += 1
        row = (self.sample_seq, time.time(), r.counts, r.speed_rpm, r.status, r.milliamps, r.angle_error, r.io)
        self.samples.append(row)
        if self._recording is not None and len(self._recording) < RECORD_MAX:
            self._recording.append(row)

    def _end_motion(self) -> None:
        self.motion, self._jog = None, None

    def _moving(self) -> bool:
        return self.readout.status in (2, 3, 4, 5, 6) and not self._stale("status", 0.5)

    def _tick(self, now: float) -> None:
        with self._lock:
            if self._jog is not None and now - self._jog_keepalive > JOG_KEEPALIVE_S:
                self._jog = None
                self._soft_stop_frames()
            busy = self.motion is not None or self._moving() or (self.routine is not None and self.routine.active)
            if busy and now - self._last_page > PAGE_TIMEOUT_S and now - self._last_page_halt > 0.5:
                self._last_page_halt = now
                self._halt_frames(f"the page has been silent for {PAGE_TIMEOUT_S:g} s")
            if (self.motion == "run" and not self._window_stop and self.mode == "bench" and self.readout.counts is not None
                    and not self.limits.bench_min <= self.readout.counts * MOTOR_DEG_PER_COUNT <= self.limits.bench_max):
                self._window_stop = True   # once: the run's stop is on its way
                self._soft_stop_frames()
                self.notice("The speed run left the bench window: stopped", "warn")
            if (self.motion in ("move", "jog", "run") and now - self._motion_since > 0.3 and self.readout.status == 1
                    and self.readout.updated.get("status", 0) > self._motion_since + 0.2 and self._jog is None):
                self._end_motion()   # stopped by status (when no second reply came)
            last = self.worker.last_reply.get(self.can_id)
            online = last is not None and now - last < OFFLINE_S
            if online and self.readout.version is None and now - self._version_asked > 2.0:
                self._version_asked = now
                self.worker.send(self.can_id, bytes([Cmd.READ_VERSION]))
            if self._online and not online:
                self._offline_since = last
                self.notice(f"Board {self.can_id} stopped answering", "warn")
            elif not self._online and online and self._offline_since is not None:
                if self.zero_deg is not None and self.mode == "joint":
                    self.zero_deg, self.reference = None, "not set"
                    self.notice("The board was silent and came back: if it lost power its encoder's zero moved, so "
                                "the reference is cleared - home it or set it again", "warn")
                self._offline_since = None
            self._online = online

    # --- stopping ---------------------------------------------------------------------------------------------------

    def _soft_stop_frames(self) -> None:
        """A ramped stop for what is running, queued ahead of everything (any thread, never blocks)."""
        acc = self._last_acc
        if self.motion == "run":
            self.worker.send(self.can_id, speed_request(0, 0, acc), priority=True)
        elif self.motion in ("home", "calibrate"):
            self.worker.send(self.can_id, emergency_stop_request(self.priority_estop), priority=True)
        else:
            self.worker.send(self.can_id, move_to_axis_request(0, 0, acc), priority=True)
        self._motion_since = time.monotonic()

    def _halt_frames(self, reason: str) -> None:
        """STOP: F7 first, then the immediate F5 stop (and F6's after a run) - all ahead of everything queued."""
        if self.routine is not None and self.routine.active:
            self.routine.cancel.set()
        run = self.motion == "run"
        self._end_motion()
        if run:
            self.worker.send(self.can_id, speed_request(0, 0, 0), priority=True)
        self.worker.send(self.can_id, move_to_axis_request(0, 0, 0), priority=True)
        self.worker.send(self.can_id, emergency_stop_request(self.priority_estop), priority=True)
        self.notice(f"STOP: {reason}", "warn")

    def halt(self, reason: str = "STOP pressed") -> bool | None:
        """Stop everything now; True when the board confirmed F7, None when it did not answer."""
        seq = self.worker.event_seq
        with self._lock:
            self._halt_frames(reason)
        reply = self.worker.wait_event(seq, lambda r: r.can_id == self.can_id and r.code == Cmd.EMERGENCY_STOP, 0.5)
        return None if reply is None else reply.status == 1

    def stop(self) -> None:
        """A ramped stop of the running motion (the Stop button beside the motion controls)."""
        with self._lock:
            if self.routine is not None and self.routine.active:
                self.routine.cancel.set()
            self._jog = None
            self._soft_stop_frames()

    # --- commands ---------------------------------------------------------------------------------------------------

    def _status_command(self, data: bytes, what: str, ok: tuple[int, ...] = (1,), timeout: float = REPLY_S) -> Reply:
        reply = self.worker.request(self.can_id, data, timeout=timeout)
        if reply is None:
            raise Refused(f"{what}: no reply from board {self.can_id} - is it powered, on this id, on the bus?")
        if reply.status not in ok:
            raise Refused(f"{what}: the board answered status {reply.status}")
        return reply

    def enable(self, on: bool) -> None:
        if not on:
            with self._lock:
                if self.routine is not None and self.routine.active:
                    self.routine.cancel.set()
                self._end_motion()
        self._status_command(bytes([Cmd.ENABLE, int(on)]), "enable" if on else "disable")

    def _start_motion(self, kind: str, data: bytes, acc: int, *, routine: bool = False) -> int:
        """Send a motion frame; returns the event seq from before it (to wait for its end). Raises Refused."""
        with self._lock:
            if not routine:
                if self.routine is not None and self.routine.active:
                    raise Refused(f"a test is running ({self.routine.title}): cancel it first")
                if self.motion in ("home", "calibrate"):
                    raise Refused(f"the board is {'homing' if self.motion == 'home' else 'calibrating'}: wait or STOP")
            self.motion, self._motion_since, self._last_acc = kind, time.monotonic(), acc
            self._window_stop = False
        seq = self.worker.event_seq
        reply = self.worker.request(self.can_id, data, timeout=REPLY_S)
        if reply is None or reply.status not in (1, 2):
            with self._lock:
                self._end_motion()
            if reply is None:
                raise Refused(f"no reply to the move from board {self.can_id}")
            raise Refused(f"the board refused the move (status {reply.status}): is the motor enabled and the work "
                          f"mode a bus mode (SR_vFOC)?")
        if reply.status == 2:
            with self._lock:
                self._end_motion()
        return seq

    def move_to(self, value: float, speed: float, acc: int, *, routine: bool = False) -> int:
        counts, rpm = self.check(value, speed, acc)
        return self._start_motion("move", move_to_axis_request(counts, rpm, acc), acc, routine=routine)

    def move_by(self, delta: float, speed: float, acc: int) -> int:
        now = self.value_now()
        if now is None or self._stale("counts", 0.5):
            raise Refused("no position to step from (joint mode needs its reference; is the board answering?)")
        return self.move_to(now + delta, speed, acc)

    def move_counts_relative(self, delta_counts: int, speed: float, acc: int, *, routine: bool = False) -> int:
        """A relative move by encoder counts - the direction check's, which works before the reference is set."""
        rpm = self.speed_to_rpm(speed)
        return self._start_motion("move", move_relative_axis_request(delta_counts, rpm, acc), acc, routine=routine)

    def jog(self, direction: int, speed: float, acc: int) -> str:
        """Start a jog toward the window's edge, or keep the running one alive (the page repeats this every 0.1 s)."""
        if direction not in (1, -1):
            raise Refused("jog direction is +1 or -1")
        now_t = time.monotonic()
        with self._lock:
            if self._jog == direction:
                self._jog_keepalive = now_t
                return "jogging"
            lo, hi = self.window()
            edge = hi if direction > 0 else lo
            here = self.value_now()
            if here is not None and (here >= edge - 1e-6 if direction > 0 else here <= edge + 1e-6):
                raise Refused(f"already at the {'upper' if direction > 0 else 'lower'} end ({edge:g})")
            counts, rpm = self.check(edge, speed, acc)
            self._jog, self._jog_keepalive = direction, now_t
        self._start_motion("jog", move_to_axis_request(counts, rpm, acc), acc)
        return "jog started"

    def jog_release(self) -> None:
        with self._lock:
            if self._jog is not None:
                self._jog = None
                self._soft_stop_frames()

    def run(self, cw: bool, rpm: float, acc: int) -> None:
        """Bench mode only: a constant-speed run (F6), stopped by the window, the page's silence or a stop."""
        with self._lock:
            if self.mode != "bench":
                raise Refused("a constant-speed run is bench mode's only: a joint's moves stay position moves, so "
                              "a lost page or a crashed dashboard leaves it inside its limits")
            value = self.value_now()
            if value is None:
                raise Refused("no position reading yet")
            if not self.limits.bench_min <= value <= self.limits.bench_max:
                raise Refused("the motor is outside the bench window")
            whole = self.speed_to_rpm(rpm)
            if not 0 <= acc <= MAX_ACC:
                raise Refused(f"acc {acc} outside 0..{MAX_ACC}")
        self._start_motion("run", speed_request(int(cw), whole, acc), acc)

    def set_zero(self) -> None:
        """0x92: the current position becomes the encoder's zero; joint mode's reference follows."""
        with self._lock:
            self._require_idle("setting the zero")
            deg = self.value_now() if self.mode == "joint" else None
        self._status_command(bytes([Cmd.SET_ZERO]), "set zero")
        with self._lock:
            self.readout.counts = 0
            if deg is not None:
                self.zero_deg = deg
                self.reference = f"kept through set zero: the encoder's zero is {deg:g} deg"

    def release_stall(self) -> None:
        self._status_command(bytes([Cmd.RELEASE_STALL]), "release stall")

    def write_setting(self, code: int, value: int | None = None) -> str:
        if code in ACTIONS:
            with self._lock:
                self._require_idle(ACTIONS[code])
            reply = self.worker.request(self.can_id, bytes([code]), timeout=0.5)
            if code == 0x3F:
                self.notice("Factory defaults restored: the board's CAN id is 1 again and its work mode CR_vFOC "
                            "(a pulse mode) - set SR_vFOC and the id again", "warn")
            if reply is None:
                return f"{ACTIONS[code]} sent (no reply - the board may be restarting)"
            return f"{ACTIONS[code]}: status {reply.status}"
        if code not in SETTINGS:
            raise Refused(f"0x{code:02X} is not a setting the dashboard writes")
        name, size, lo, hi = SETTINGS[code]
        if value is None or not lo <= value <= hi:
            raise Refused(f"{name}: {value} outside {lo}..{hi}")
        with self._lock:
            self._require_idle(f"setting the {name}")
        data = bytes([code]) + value.to_bytes(size, "big")
        self._status_command(data, f"set {name}", ok=(1, 2) if code == 0x83 else (1,), timeout=0.5)
        if code == 0x8B:
            with self._lock:
                self.can_id = value
                self.worker.driver(value)
                self.worker.set_poll(value, *self._poll_profile)
            self.notice(f"The board's CAN id is {value} now; the dashboard follows it. config.JOINTS has "
                        f"{self.joint_config.name} at {self.joint_config.can_id}")
        return f"{name} set to {value}"

    def read_settings(self) -> dict[str, Any]:
        out: dict[str, Any] = {}
        for code in READABLE:
            reply = self.worker.request(self.can_id, bytes([Cmd.READ_PARAM, code]), code=code, timeout=0.2)
            if reply is None:
                out[f"{code:02X}"] = None
                continue
            raw = bytes(reply.data[1:])
            value = None if raw == b"\xff\xff" or not raw else int.from_bytes(raw, "big")
            out[f"{code:02X}"] = {"raw": raw.hex(" "), "value": value}
        return out

    def raw(self, text: str) -> dict[str, Any]:
        try:
            data = bytes.fromhex(text.replace(",", " "))
        except ValueError as e:
            raise Refused(f"not hex bytes: {text!r}") from e
        if not 1 <= len(data) <= 7:
            raise Refused("1-7 bytes: the code and its data, without the CAN id or the check byte")
        if data[0] in RAW_REFUSED:
            raise Refused(f"0x{data[0]:02X} ({RAW_REFUSED[data[0]]}) moves the motor past the dashboard's checks: "
                          "use the motion panel or a test")
        reply = self.worker.request(self.can_id, data, timeout=0.4)
        return {"sent": data.hex(" "), "reply": None if reply is None else reply.data.hex(" ")}

    def scan(self, first: int, last: int) -> list[dict[str, Any]]:
        """0x40 to every id in first..last; the ones that answer. Boards on one id answer as one."""
        if not 1 <= first <= last <= 0x7FF or last - first > 255:
            raise Refused("scan 1..2047, at most 256 ids at a time")
        futures = {i: self.worker.submit(i, bytes([Cmd.READ_VERSION]), timeout=0.4) for i in range(first, last + 1)}
        found = []
        for i, future in futures.items():
            reply = future.result(3.0)
            if reply is None:
                continue
            v = decode_version(reply)
            found.append({"can_id": i, "board": None if v is None else HARDWARE_NAMES.get(v.hardware, str(v.hardware)),
                          "firmware": None if v is None else v.firmware_text,
                          "calibrated": None if v is None else v.calibrated})
        return found

    # --- recording --------------------------------------------------------------------------------------------------

    def set_recording(self, on: bool) -> None:
        with self._lock:
            if on and self._recording is None:
                self._recording = []
            elif not on and self._recording is not None:
                self._last_recording, self._recording = self._recording, None

    def recording_csv(self) -> str:
        with self._lock:
            rows = list(self._recording if self._recording is not None else self._last_recording)
            return self.samples_csv(rows)

    def samples_csv(self, rows: list[tuple]) -> str:
        """Samples as CSV: time, counts, the position in the mode's unit, speed, status, current, error, IN_1."""
        buf = io.StringIO()
        out = csv.writer(buf)
        unit = "joint_deg" if self.mode == "joint" else "motor_deg"
        out.writerow(["time_s", "counts", unit, "speed_rpm", "status", "milliamps", "angle_error_motor_deg", "in1"])
        t0 = rows[0][1] if rows else 0.0
        for _, t, counts, rpm, status, ma, err, io_bits in rows:
            value = None if counts is None else self.to_value(counts)
            out.writerow([f"{t - t0:.4f}", counts, "" if value is None else f"{value:.4f}", rpm,
                          STATUS_NAMES.get(status, status) if status is not None else "", ma,
                          "" if err is None else f"{err * 360 / 51200:.4f}",
                          "" if io_bits is None else io_bits & 1])
        return buf.getvalue()

    def samples_since(self, seq: int) -> list[tuple]:
        with self._lock:
            return [s for s in self.samples if s[0] > seq]

    @contextmanager
    def poll_profile(self, fast: tuple[int, ...], slow: tuple[int, ...], fast_s: float) -> Iterator[None]:
        """Poll other reads, or faster, for a while (a test that needs them)."""
        with self._lock:
            before, self._poll_profile = self._poll_profile, (fast, slow, fast_s)
            self.worker.set_poll(self.can_id, *self._poll_profile)
        try:
            yield
        finally:
            with self._lock:
                self._poll_profile = before
                self.worker.set_poll(self.can_id, *before)

    # --- the page's view --------------------------------------------------------------------------------------------

    def state(self, samples_after: int = 0, log_after: int = 0, notices_after: int = 0) -> dict[str, Any]:
        with self._lock:
            r = self.readout
            now = time.monotonic()
            cfg = self.joint_config
            joint = self.joint()
            fresh = {k: now - t < 1.0 for k, t in r.updated.items()}
            routine = None if self.routine is None else self.routine.snapshot()
            samples = [s for s in self.samples if s[0] > samples_after]
            if samples_after == 0:
                samples = samples[-3000:]
            return {
                "label": self.label,
                "online": self._online,
                "bus_error": self.worker.bus_error,
                "mode": self.mode,
                "joint": cfg.name,
                "joints": [{"name": c.name, "can_id": c.can_id, "gear_ratio": c.gear_ratio, "sign": c.sign,
                            "limits": c.limits_deg, "home_deg": c.home_deg} for c in config.JOINTS],
                "can_id": self.can_id,
                "config_can_id": cfg.can_id,
                "gear_ratio": cfg.gear_ratio,
                "sign": self.sign,
                "config_sign": cfg.sign,
                "hard_limits": cfg.limits_deg,
                "home_deg": cfg.home_deg,
                "reference": self.reference,
                "zero_deg": self.zero_deg,
                "counts_per_joint_deg": joint.counts_per_deg,
                "counts_per_unit": self.counts_per_unit(),
                "max_joint_speed": self.max_joint_speed,
                "bench_rpm_ceiling": BENCH_RPM_CEILING,
                "limits": asdict(self.limits),
                "window": self.window(),
                "speed_cap": self.speed_cap(),
                "motion": self.motion,
                "jog": self._jog,
                "priority_estop": self.priority_estop,
                "readout": {
                    "counts": r.counts, "value": None if r.counts is None else self.to_value(r.counts),
                    "speed_rpm": r.speed_rpm, "status": r.status,
                    "status_name": None if r.status is None else STATUS_NAMES.get(r.status, f"unknown ({r.status})"),
                    "enabled": r.enabled, "stalled": r.stalled, "io": r.io,
                    "millivolts": r.millivolts, "milliamps": r.milliamps,
                    "angle_error_deg": None if r.angle_error is None else r.angle_error * 360 / 51200,
                    "home_status": r.home_status, "version": r.version, "fresh": fresh,
                },
                "skipped_reads": [f"0x{c:02X}" for c in self.worker.skipped_reads()],
                "counts_bus": dict(self.worker.counts),
                "sample_fields": ("seq", *SAMPLE_FIELDS),
                "samples": samples,
                "log": [asdict(e) for e in self.worker.log_since(log_after)][-500:],
                "notices": [n for n in self.notices if n["seq"] > notices_after],
                "recording": None if self._recording is None else len(self._recording),
                "routine": routine,
                "findings": list(self.findings),
                "log_polls": self.worker.log_polls,
                "settings": {f"{k:02X}": {"name": v[0], "lo": v[2], "hi": v[3]} for k, v in SETTINGS.items()},
                "work_modes": WORK_MODES,
            }

    # --- for the routines ---------------------------------------------------------------------------------------------

    def wait_motion_end(self, after_seq: int, timeout: float, cancel: threading.Event) -> int | None:
        """Wait until the motion started after event `after_seq` ends: its second reply (2 complete, 3 at a limit)
        or the board polling stopped. Returns that status (1 for a stop seen by polling), None on timeout or cancel."""
        start = time.monotonic()
        deadline = start + timeout
        moving = (Cmd.MOVE_TO_AXIS, Cmd.MOVE_RELATIVE_AXIS, Cmd.SPEED)
        while time.monotonic() < deadline and not cancel.is_set():
            for _, reply in self.worker.events_since(after_seq):
                if reply.can_id == self.can_id and reply.code in moving and reply.status in (2, 3):
                    with self._lock:
                        self._end_motion()
                    return reply.status
            with self._lock:
                t = self.readout.updated.get("status", 0)
                if self.readout.status == 1 and t > start + 0.25 and self.readout.speed_rpm == 0:
                    self._end_motion()
                    return 1
            time.sleep(0.01)
        return None

    def read_now(self, code: int) -> Reply | None:
        return self.worker.request(self.can_id, bytes([code]), timeout=REPLY_S)

    def fresh_counts(self) -> int | None:
        reply = self.read_now(Cmd.READ_ENCODER)
        return None if reply is None else decode_encoder(reply)

    def add_finding(self, text: str) -> None:
        with self._lock:
            self.findings.append(text)

    def set_motion(self, kind: str | None) -> None:
        with self._lock:
            self.motion, self._motion_since = kind, time.monotonic()

    def send_home(self, trigger: int, direction: int, rpm: int, end_limit: bool, hm_mode: int | None) -> int:
        """0x90 then 0x91; returns the event seq from before 0x91. Raises Refused."""
        data = bytes([Cmd.SET_HOME_PARAMS, trigger, direction]) + encode_uint16(rpm) + bytes([int(end_limit)])
        if hm_mode is not None:
            data += bytes([hm_mode])
        self._status_command(data, "set home parameters")
        self.set_motion("home")
        seq = self.worker.event_seq
        try:
            self._status_command(bytes([Cmd.GO_HOME]), "go home", ok=(1, 2))
        except Refused:
            self.set_motion(None)
            raise
        return seq
