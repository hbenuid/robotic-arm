"""The dashboard's model of the board under test: its joint, the units of each mode, the limits, the safety rules.

Two modes. **Joint mode**: joint degrees through the joint's gear ratio and sign (config.JOINTS), at no more than
`max_joint_speed` deg/s. With a reference (the joint angle at the encoder's zero: homing, or the user saying where the
joint is) the angles are the joint's; without one they count from the encoder's zero. **Bench mode**: degrees and RPM
of the motor's own shaft.

**Limits** (the user's choice: one switch, on by default): while on, every move and jog stays between a min and a max.
Each is the user's own - set from where the motor is ("here") or typed - or, if not set, the arm's limit from
config.JOINTS once joint mode has a reference; with neither, that side is open. They are kept in encoder counts, so
they stay with the physical position through a change of mode, sign or reference; 0x92 shifts them with the zero,
and homing or a power loss (the zero moves by an unknown amount) clears them. Switched off, nothing limits the
position: only the speed cap and STOP remain.

The safety rules, all on the bus thread's tick so they hold whatever the page does:
- STOP (halt) sends F7 and an immediate F5 stop (and an F6 stop after a speed run) ahead of everything queued.
- A jog aims JOG_LEAD_S of travel ahead (never past a limit) and the page's keep-alives move that target on; it stops
  (ramped) when the keep-alives stop for JOG_KEEPALIVE_S, and a crashed dashboard leaves at most that lead of travel.
- Any motion or test stops when the page has not been heard from for PAGE_TIMEOUT_S.
- A bench speed run stops when it passes a limit.
- A board that goes silent for longer than OFFLINE_S and comes back loses the reference and the user's limits: if it
  lost power, its encoder's zero moved.
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
from robotic_arm.dashboard.shaping import CURVES, Profile, Shaper
from robotic_arm.joint import Joint
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
JOG_LEAD_S = 2.0          # a jog's target runs this much travel ahead of the motor; keep-alives move it on
COUNTS_PER_RPM_S = ENCODER_COUNTS_PER_REV / 60   # encoder counts per second at 1 RPM
PAGE_TIMEOUT_S = 2.0      # motion and tests stop when the page is silent this long (it pings every 0.25 s)
OFFLINE_S = 1.0           # no reply for this long: the board is shown offline
REPLY_S = 0.15            # the dashboard's commands wait this long for their reply
BENCH_RPM_CEILING = 1000  # the manual: no immediate stop (F7, acc 0) above 1000 RPM
DEFAULT_JOINT_SPEED_CAP = 10.0
MOTOR_DEG_PER_COUNT = 360.0 / ENCODER_COUNTS_PER_REV
MOTION_CODES = (Cmd.MOVE_RELATIVE_AXIS, Cmd.MOVE_TO_AXIS, Cmd.SPEED)   # what a stop purges from the queue
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
    0x89: ("interpolation (MPlyer)", 1, 0, 1),            # 5.2.9: 256-step interpolation, less noise at low speed
    0x8B: ("CAN id", 2, 1, 0x7FF),
    0x98: ("heartbeat, ms (0 = off)", 4, 0, 0xFFFF_FFFF),
    0x9B: ("holding current (0-8 = 10-90 %)", 1, 0, 8),   # 5.2.4: OPEN and CLOSE modes only, not vFOC
}
ACTIONS = {0x3D: "release stall", 0x41: "restart", 0x3F: "restore factory defaults"}
READABLE = (0x82, 0x83, 0x84, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x9B, 0x9E)   # 0x00 read-backs the Settings panel shows
# 5.3 (V1.0.8): the position loop's gains, 0-1024 each; code, then 00 + Kp Ki or 01 + Kd Kv. The manual's defaults.
PID_CODES = {"vfoc": 0x96, "close": 0x97}
PID_DEFAULTS = {"vfoc": {"kp": 220, "ki": 100, "kd": 270, "kv": 320}, "close": {"kp": 200, "ki": 80, "kd": 250, "kv": 300}}
WORK_MODES = {0: "CR_OPEN", 1: "CR_CLOSE", 2: "CR_vFOC", 3: "SR_OPEN", 4: "SR_CLOSE", 5: "SR_vFOC"}


class Refused(Exception):
    """A command the dashboard did not send, or one the board refused; the message says why."""


@dataclass
class Limits:
    """The limits. `on`: the position limits apply. count_lo / count_hi: the user's own bounds in encoder counts (the
    motor must stay >= count_lo and <= count_hi); None: not set. The speed caps always apply."""

    joint_speed: float
    on: bool = True
    count_lo: int | None = None
    count_hi: int | None = None
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
    cmd_rpm: float | None = None      # a shaped move's commanded speed, RPM signed with the encoder's count
    updated: dict[str, float] = field(default_factory=dict)


SAMPLE_FIELDS = ("t", "counts", "speed_rpm", "status", "milliamps", "angle_error", "io", "cmd_rpm")


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
        self.limits = Limits(self.max_joint_speed)
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
        self._jog_goal: int | None = None       # the running jog's F5 target, counts
        self._jog_rpm = 0
        self._window_stop = False
        self._last_page = time.monotonic()
        self._last_page_halt = 0.0
        self._online = False
        self._offline_since: float | None = None
        self._version_asked = 0.0
        self._last_recording: list[tuple] = []
        self.routine = None                     # a routines.Routine while one runs or after it ends
        self.profile = Profile()
        self._shape_gen = 0                     # bumped by every stop: a running Shaper sends nothing after it
        self._shaper: Shaper | None = None
        self._fast_s = 0.02
        self._poll_profile = (FAST_POLL, SLOW_POLL, self._fast_s)
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

    def to_value(self, counts: int) -> float:
        """`counts` in the mode's unit: motor degrees, the joint's angle, or (no reference) joint degrees from the
        encoder's zero."""
        with self._lock:
            if self.mode == "bench":
                return counts * MOTOR_DEG_PER_COUNT
            joint = self.joint()
            if self.zero_deg is None:
                return self.sign * counts / joint.counts_per_deg
            return joint.counts_to_deg(counts)

    def value_to_counts(self, value: float) -> int:
        """The mode's unit -> encoder counts (to_value's inverse)."""
        with self._lock:
            if self.mode == "bench":
                return round(value / MOTOR_DEG_PER_COUNT)
            joint = self.joint()
            if self.zero_deg is None:
                return round(self.sign * value * joint.counts_per_deg)
            return joint.deg_to_counts(value)

    def value_now(self) -> float | None:
        counts = self.readout.counts
        return None if counts is None else self.to_value(counts)

    def _value_dir(self) -> int:
        """+1 when the mode's value grows with the encoder's count, -1 when it falls (joint mode, sign -1)."""
        return 1 if self.mode == "bench" or self.sign > 0 else -1

    def _arm_bounds(self) -> tuple[int | None, int | None]:
        """config.JOINTS' limits in counts (lo, hi): joint mode with a reference only."""
        if self.mode != "joint" or self.zero_deg is None:
            return None, None
        joint = self.joint()
        a, b = (joint.deg_to_counts(d) for d in self.joint_config.limits_deg)
        return min(a, b), max(a, b)

    def _bounds(self, *, even_off: bool = False) -> tuple[int | None, int | None, str | None, str | None]:
        """(lo, hi, lo's source, hi's source) in counts: the user's bound, else the arm's, else None (open); all
        None while the limits are off (unless `even_off`, for showing them)."""
        if not self.limits.on and not even_off:
            return None, None, None, None
        arm_lo, arm_hi = self._arm_bounds()
        lo, lo_src = (self.limits.count_lo, "yours") if self.limits.count_lo is not None else (arm_lo, "arm" if arm_lo is not None else None)
        hi, hi_src = (self.limits.count_hi, "yours") if self.limits.count_hi is not None else (arm_hi, "arm" if arm_hi is not None else None)
        return lo, hi, lo_src, hi_src

    def limit_view(self) -> dict[str, Any]:
        """The limits as the page shows them: min and max in the mode's unit, each with its source."""
        with self._lock:
            lo, hi, lo_src, hi_src = self._bounds(even_off=True)
            low = (None if lo is None else self.to_value(lo), lo_src)
            high = (None if hi is None else self.to_value(hi), hi_src)
            mn, mx = (low, high) if self._value_dir() > 0 else (high, low)
            return {"on": self.limits.on, "min": mn[0], "min_src": mn[1], "max": mx[0], "max_src": mx[1]}

    def window(self) -> tuple[float | None, float | None]:
        """The travel the limits allow now, in the mode's unit; None for an open side (or the limits off)."""
        with self._lock:
            if not self.limits.on:
                return None, None
            view = self.limit_view()
            return view["min"], view["max"]

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
            if not math.isfinite(value):
                raise Refused(f"not a position: {value}")
            rpm = self.speed_to_rpm(speed)
            counts = self.value_to_counts(value)
            if abs(counts) > INT24_MAX:
                raise Refused(f"{value:g} is past the board's +/-{INT24_MAX} counts")
            lo, hi, _, _ = self._bounds()
            if (lo is not None and counts < lo) or (hi is not None and counts > hi):
                mn, mx = self.window()
                span = f"{'open' if mn is None else f'{mn:.2f}'} .. {'open' if mx is None else f'{mx:.2f}'}"
                raise Refused(f"{value:g} is outside the limits ({span}); switch the limits off to go past them")
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
                self.limits = replace(self.limits, on=True, count_lo=None, count_hi=None)
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
            self.notice("Bench mode: motor degrees and RPM; the arm's own limits do not apply here, your own do while "
                        "the limits are on", "warn")

    def set_speed_limits(self, joint_speed: float | None = None, bench_rpm: float | None = None) -> None:
        """The speed caps: joint mode's deg/s (never above max_joint_speed), bench mode's RPM."""
        with self._lock:
            new = self.limits
            if joint_speed is not None:
                if not 0 < joint_speed <= self.max_joint_speed:
                    raise Refused(f"joint speed limit 0..{self.max_joint_speed:g} deg/s (the dashboard's cap; "
                                  f"--max-joint-speed raises it)")
                new = replace(new, joint_speed=float(joint_speed))
            if bench_rpm is not None:
                if not 1 <= bench_rpm <= BENCH_RPM_CEILING:
                    raise Refused(f"bench speed limit 1..{BENCH_RPM_CEILING} RPM")
                new = replace(new, bench_rpm=float(bench_rpm))
            self.limits = new

    def set_profile(self, **values: Any) -> None:
        """How moves are shaped (shaping.Profile): kind 'board' or 'scurve', curve, ease_s, rate_hz, follow_acc. It
        applies to the next move or jog."""
        with self._lock:
            new = replace(self.profile, **{k: v for k, v in values.items() if k in asdict(self.profile) and v is not None})
            new.ease_s, new.rate_hz, new.follow_acc = float(new.ease_s), float(new.rate_hz), int(new.follow_acc)
            if new.kind not in ("board", "scurve"):
                raise Refused("the profile is board or scurve")
            if new.curve not in CURVES:
                raise Refused(f"the curve is one of {', '.join(CURVES)}")
            if not 0.05 <= new.ease_s <= 5:
                raise Refused("ease time 0.05..5 s")
            if not 5 <= new.rate_hz <= 100:
                raise Refused("update rate 5..100 per second")
            if not 1 <= new.follow_acc <= MAX_ACC:
                raise Refused(f"follow ramp 1..{MAX_ACC}")
            self.profile = new

    def set_poll_period(self, ms: float) -> None:
        """How often the encoder, speed and status are read (the graph's detail): 5, 10 or 20 ms."""
        if ms not in (5, 10, 20):
            raise Refused("the poll period is 5, 10 or 20 ms")
        with self._lock:
            default = self._poll_profile == (FAST_POLL, SLOW_POLL, self._fast_s)
            self._fast_s = ms / 1000
            if default:
                self._poll_profile = (FAST_POLL, SLOW_POLL, self._fast_s)
                self.worker.set_poll(self.can_id, *self._poll_profile)

    def write_pid(self, loop: str, **gains: int) -> str:
        """5.3 (firmware V1.0.8+): the position loop's Kp, Ki (sent together) and Kd, Kv (together), 0-1024 each."""
        if loop not in PID_CODES:
            raise Refused("the loop is vfoc or close")
        with self._lock:
            self._require_idle("changing the PID gains")
        code, sent = PID_CODES[loop], []
        for sub, (a, b) in ((0, ("kp", "ki")), (1, ("kd", "kv"))):
            if gains.get(a) is None and gains.get(b) is None:
                continue
            if gains.get(a) is None or gains.get(b) is None:
                raise Refused(f"{a} and {b} go together")
            for name in (a, b):
                if not 0 <= int(gains[name]) <= 1024:
                    raise Refused(f"{name} {gains[name]} outside 0..1024")
            data = bytes([code, sub]) + int(gains[a]).to_bytes(2, "big") + int(gains[b]).to_bytes(2, "big")
            self._status_command(data, f"set {a}/{b} ({loop}; firmware V1.0.8+)", timeout=0.5)
            sent.append(f"{a} {gains[a]}, {b} {gains[b]}")
        return f"{loop}: " + "; ".join(sent) if sent else "nothing to write"

    def read_pid(self, loop: str) -> dict[str, str | None]:
        """5.3's read-backs (00 code 00 / 00 code 01), as the raw bytes the board sends."""
        if loop not in PID_CODES:
            raise Refused("the loop is vfoc or close")
        code, out = PID_CODES[loop], {}
        for sub, name in ((0, "kp_ki"), (1, "kd_kv")):
            reply = self.worker.request(self.can_id, bytes([Cmd.READ_PARAM, code, sub]), code=code, timeout=0.3)
            out[name] = None if reply is None else bytes(reply.data[1:]).hex(" ")
        return out

    # --- shaped moves (shaping.Shaper) ------------------------------------------------------------------------------

    def _stop_shaper(self) -> None:
        """Under the lock: no shaped speed is sent after this."""
        self._shape_gen += 1
        self._shaper = None
        self.readout.cmd_rpm = None

    def _start_shaper(self, start: int, vmax: float, *, target: int | None = None, jog_dir: int | None = None) -> None:
        with self._lock:
            shaper = Shaper(self, self._shape_gen, start, vmax, target=target, jog_dir=jog_dir, lead_s=JOG_LEAD_S)
            self._shaper = shaper
        shaper.start()

    def _shaper_done(self, shaper: Shaper) -> None:
        with self._lock:
            if self._shaper is shaper:
                self._shaper = None

    def set_limits_on(self, on: bool) -> None:
        with self._lock:
            self.limits = replace(self.limits, on=bool(on))
        if not on:
            self.notice("Limits off: nothing limits the position now - only the speed cap and STOP", "warn")

    def set_limit(self, side: str, value: float | None = None, *, here: bool = False) -> None:
        """Set the min or max limit: where the motor is now (`here`), at `value` (the mode's unit), or clear it
        (`value` None) - then the arm's own applies, if there is one."""
        if side not in ("min", "max"):
            raise Refused("a limit is min or max")
        with self._lock:
            if here:
                counts = self.readout.counts
                if counts is None or self._stale("counts", 0.5):
                    raise Refused("no fresh encoder reading: is the board answering?")
            elif value is None:
                counts = None
            else:
                if not math.isfinite(value):
                    raise Refused(f"not a position: {value}")
                counts = self.value_to_counts(value)
                if abs(counts) > INT24_MAX:
                    raise Refused(f"{value:g} is past the board's +/-{INT24_MAX} counts")
            key = "count_lo" if (side == "min") == (self._value_dir() > 0) else "count_hi"
            new = replace(self.limits, **{key: counts})
            lo, hi, _, _ = self._bounds_of(new)
            if lo is not None and hi is not None and lo >= hi:
                raise Refused("the min limit must be below the max")
            self.limits = new

    def _bounds_of(self, limits: Limits) -> tuple[int | None, int | None, str | None, str | None]:
        before, self.limits = self.limits, limits
        try:
            return self._bounds(even_off=True)
        finally:
            self.limits = before

    def _clear_user_limits(self, why: str) -> None:
        if self.limits.count_lo is not None or self.limits.count_hi is not None:
            self.limits = replace(self.limits, count_lo=None, count_hi=None)
            self.notice(f"Your limits are cleared: {why} - set them again", "warn")

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

    def homed(self) -> None:
        """Homing succeeded: the encoder's zero is the switch's edge - joint mode's reference is the joint's home_deg
        there - and your limits, counted from the old zero, are cleared."""
        with self._lock:
            if self.mode == "joint":
                self.zero_deg = self.joint_config.home_deg
                self.reference = f"homed: the switch's edge is home_deg {self.joint_config.home_deg:g} deg"
            self._clear_user_limits("homing moved the encoder's zero")

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
                  and reply.status in (2, 3) and self.motion in ("move", "jog", "run", "step")
                  and self._shaper is None):   # a shaped move ends when its Shaper does (an update's 2 is not it)
                if reply.status == 3:
                    self.notice("The board stopped the move at a limit (status 3)", "warn")
                self._end_motion()

    def _add_sample(self) -> None:
        r = self.readout
        self.sample_seq += 1
        row = (self.sample_seq, time.time(), r.counts, r.speed_rpm, r.status, r.milliamps, r.angle_error, r.io,
               r.cmd_rpm)
        self.samples.append(row)
        if self._recording is not None and len(self._recording) < RECORD_MAX:
            self._recording.append(row)

    def _end_motion(self) -> None:
        self.motion, self._jog, self._jog_goal = None, None, None

    def _moving(self) -> bool:
        return self.readout.status in (2, 3, 4, 5, 6) and not self._stale("status", 0.5)

    def _tick(self, now: float) -> None:
        with self._lock:
            if self._jog is not None and now - self._jog_keepalive > JOG_KEEPALIVE_S:
                self._jog = None
                self._soft_stop_frames()   # the page lost: the board's own ramp, not an eased stop
            busy = self.motion is not None or self._moving() or (self.routine is not None and self.routine.active)
            if busy and now - self._last_page > PAGE_TIMEOUT_S and now - self._last_page_halt > 0.5:
                self._last_page_halt = now
                self._halt_frames(f"the page has been silent for {PAGE_TIMEOUT_S:g} s")
            if self.motion == "run" and not self._window_stop and self.readout.counts is not None:
                lo, hi, _, _ = self._bounds()
                if (lo is not None and self.readout.counts < lo) or (hi is not None and self.readout.counts > hi):
                    self._window_stop = True   # once: the run's stop is on its way
                    self._soft_stop_frames()
                    self.notice("The speed run reached a limit: stopped", "warn")
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
                if self.zero_deg is not None:
                    self.zero_deg, self.reference = None, "not set"
                    self.notice("The board was silent and came back: if it lost power its encoder's zero moved, so "
                                "the reference is cleared - home it or set it again", "warn")
                self._clear_user_limits("the board was silent and came back (its encoder's zero may have moved)")
                self._offline_since = None
            self._online = online

    # --- stopping ---------------------------------------------------------------------------------------------------

    def _soft_stop_frames(self) -> None:
        """A ramped stop for what is running, queued ahead of everything (any thread, never blocks); queued moves
        and a shaped move's further speeds are dropped first."""
        self._stop_shaper()
        self.worker.purge(self.can_id, MOTION_CODES)
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
        self._stop_shaper()
        self.worker.purge(self.can_id, MOTION_CODES)   # no move queued before the stop goes out after it
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
                self._stop_shaper()
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
        """A move to `value` (the mode's unit): straight to the board (its own ramp, `acc`), or shaped (the S-curve
        profile: a slow first F5, then the Shaper's updates up to `speed`)."""
        counts, rpm = self.check(value, speed, acc)
        with self._lock:
            self._stop_shaper()
            shaped, start = self.profile.kind == "scurve", self.readout.counts
        if not shaped or start is None:
            return self._start_motion("move", move_to_axis_request(counts, rpm, acc), acc, routine=routine)
        follow = self.profile.follow_acc
        seq = self._start_motion("move", move_to_axis_request(counts, max(1, round(0.03 * rpm)), follow), follow,
                                 routine=routine)
        self._start_shaper(start, rpm, target=counts)
        return seq

    def move_by(self, delta: float, speed: float, acc: int) -> int:
        now = self.value_now()
        if now is None or self._stale("counts", 0.5):
            raise Refused("no position to step from: is the board answering?")
        return self.move_to(now + delta, speed, acc)

    def move_counts_relative(self, delta_counts: int, speed: float, acc: int, *, routine: bool = False) -> int:
        """A relative move by encoder counts - the direction check's, which works before the reference is set."""
        rpm = self.speed_to_rpm(speed)
        return self._start_motion("move", move_relative_axis_request(delta_counts, rpm, acc), acc, routine=routine)

    def _jog_target(self, here: int, direction: int, rpm: int, acc: int) -> int | None:
        """A jog's F5 target in counts: JOG_LEAD_S of travel ahead (and at least twice the braking distance, so it
        does not slow for it), never past a limit; None when already at the limit that way."""
        cdir = direction * self._value_dir()
        v = rpm * COUNTS_PER_RPM_S
        brake = 0.0 if acc == 0 else v * v / (2 * COUNTS_PER_RPM_S / ((256 - acc) * 50e-6))   # 9.1's ramp
        target = here + cdir * max(v * JOG_LEAD_S, 2 * brake, 64)
        lo, hi, _, _ = self._bounds()
        if cdir > 0 and hi is not None:
            target = min(target, hi)
        if cdir < 0 and lo is not None:
            target = max(target, lo)
        target = max(-INT24_MAX, min(INT24_MAX, round(target)))
        return None if (target - here) * cdir <= 0 else target

    def jog(self, direction: int, speed: float, acc: int) -> str:
        """Start a jog (+1 / -1 in the mode's unit), or keep the running one alive - the page repeats this every
        0.1 s, and each keep-alive moves the target on once a quarter of its lead is used (or the speed changed)."""
        if direction not in (1, -1):
            raise Refused("jog direction is +1 or -1")
        if not 0 <= acc <= MAX_ACC:
            raise Refused(f"acc {acc} outside 0..{MAX_ACC}")
        rpm = self.speed_to_rpm(speed)
        now_t = time.monotonic()
        with self._lock:
            here = self.readout.counts
            if here is None or self._stale("counts", 0.5):
                raise Refused("no fresh encoder reading: is the board answering?")
            shaper = self._shaper if self._shaper is not None and self._shaper.jog_dir is not None else None
            if shaper is not None and shaper.jog_dir == direction * self._value_dir():
                self._jog, self._jog_keepalive = direction, now_t   # a shaped jog: kept alive, sped up or resumed
                shaper.vmax = float(rpm)
                shaper.resume()
                return "jogging"
            if self.profile.kind == "scurve":
                follow = self.profile.follow_acc
                target = self._jog_target(here, direction, rpm, follow)
                if target is None:
                    raise Refused(f"at the {'max' if direction > 0 else 'min'} limit (switch the limits off to go on)")
                self._stop_shaper()
                self._jog, self._jog_keepalive, self._jog_goal, self._jog_rpm = direction, now_t, target, rpm
                shaped = True
            else:
                shaped = False
        if shaped:
            self._start_motion("jog", move_to_axis_request(target, max(1, round(0.03 * rpm)), follow), follow)
            with self._lock:
                self._jog = direction   # _start_motion's reply path may have cleared it
            self._start_shaper(here, rpm, jog_dir=direction * self._value_dir())
            return "jog started"
        with self._lock:
            if self._jog == direction:
                self._jog_keepalive = now_t
                new, goal = self._jog_target(here, direction, rpm, acc), self._jog_goal
                if new is None or goal is None or (rpm == self._jog_rpm and abs(new - goal) * 4 < abs(new - here)):
                    return "jogging"
                self._jog_goal, self._jog_rpm, self._last_acc = new, rpm, acc
                retarget = move_to_axis_request(new, rpm, acc)
            else:
                target = self._jog_target(here, direction, rpm, acc)
                if target is None:
                    raise Refused(f"at the {'max' if direction > 0 else 'min'} limit (switch the limits off to go on)")
                self._jog, self._jog_keepalive, self._jog_goal, self._jog_rpm = direction, now_t, target, rpm
                retarget = None
        if retarget is not None:
            self.worker.send(self.can_id, retarget)   # 11.4 note 2: F5 takes a new target while it runs
            return "jogging"
        self._start_motion("jog", move_to_axis_request(target, rpm, acc), acc)
        return "jog started"

    def jog_release(self) -> None:
        with self._lock:
            if self._jog is not None:
                self._jog = None
                if self._shaper is not None and self._shaper.jog_dir is not None:
                    self._shaper.release()   # eases down along the curve, then stops
                else:
                    self._soft_stop_frames()

    def run(self, cw: bool, rpm: float, acc: int) -> None:
        """Bench mode only: a constant-speed run (F6), stopped by a limit, the page's silence or a stop."""
        with self._lock:
            if self.mode != "bench":
                raise Refused("a constant-speed run is bench mode's only: a joint's moves stay position moves, so "
                              "a lost page or a crashed dashboard leaves at most a jog's lead of travel")
            counts = self.readout.counts
            if counts is None:
                raise Refused("no position reading yet")
            lo, hi, _, _ = self._bounds()
            if (lo is not None and counts < lo) or (hi is not None and counts > hi):
                raise Refused("the motor is outside the limits")
            whole = self.speed_to_rpm(rpm)
            if not 0 <= acc <= MAX_ACC:
                raise Refused(f"acc {acc} outside 0..{MAX_ACC}")
        self._start_motion("run", speed_request(int(cw), whole, acc), acc)

    def set_zero(self) -> None:
        """0x92: the current position becomes the encoder's zero; the reference and your limits follow it."""
        with self._lock:
            self._require_idle("setting the zero")
            shift = self.readout.counts
            if shift is None or self._stale("counts", 0.5):
                raise Refused("no fresh encoder reading: is the board answering?")
            deg = self.to_value(shift) if self.mode == "joint" and self.zero_deg is not None else None
        self._status_command(bytes([Cmd.SET_ZERO]), "set zero")
        with self._lock:
            self.readout.counts = 0
            lim = self.limits
            self.limits = replace(lim, count_lo=None if lim.count_lo is None else lim.count_lo - shift,
                                  count_hi=None if lim.count_hi is None else lim.count_hi - shift)
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
        out.writerow(["time_s", "counts", unit, "speed_rpm", "status", "milliamps", "angle_error_motor_deg", "in1",
                      "commanded_rpm"])
        t0 = rows[0][1] if rows else 0.0
        for _, t, counts, rpm, status, ma, err, io_bits, cmd in rows:
            value = None if counts is None else self.to_value(counts)
            out.writerow([f"{t - t0:.4f}", counts, "" if value is None else f"{value:.4f}", rpm,
                          STATUS_NAMES.get(status, status) if status is not None else "", ma,
                          "" if err is None else f"{err * 360 / 51200:.4f}",
                          "" if io_bits is None else io_bits & 1, "" if cmd is None else f"{cmd:.2f}"])
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
                "limits": {**self.limit_view(), "joint_speed": self.limits.joint_speed,
                           "bench_rpm": self.limits.bench_rpm},
                "profile": asdict(self.profile),
                "curves": list(CURVES),
                "shaping": self._shaper is not None,
                "poll_ms": round(self._fast_s * 1000),
                "pid_defaults": PID_DEFAULTS,
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
        with self._lock:
            shaper = self._shaper
        if shaper is not None:   # a shaped move: its Shaper ends at the target, then the board's own reply
            while not shaper.done.wait(0.01):
                if cancel.is_set() or time.monotonic() > deadline:
                    return None
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
