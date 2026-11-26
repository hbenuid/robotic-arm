"""The dashboard's tests: each runs on its own thread, drives the board through Bench (so every move is checked
there) and fills a table, a summary, a chart and findings for the page.

One runs at a time. STOP, the page's silence or Cancel sets its `cancel`; a routine that ends any way but done stops
the motor (ramped) on its way out.
"""

from __future__ import annotations

import csv
import io
import itertools
import math
import queue
import statistics
import threading
import time
from collections.abc import Callable
from typing import Any

from robotic_arm.dashboard.bench import MOTOR_DEG_PER_COUNT, WORK_MODES, Bench, Refused
from robotic_arm.motor_driver import HARDWARE_NAMES, Cmd, decode_power, decode_version


class Cancelled(Exception):
    pass


class Routine:
    """One test run: its parameters, progress, a question waiting for the user, and its results."""

    def __init__(self, kind: str, title: str, params: dict[str, Any]):
        self.kind, self.title, self.params = kind, title, params
        self.cancel = threading.Event()
        self.status = "running"            # running, waiting (a question), done, failed, cancelled
        self.progress = ""
        self.question: dict[str, Any] | None = None
        self._answers: queue.Queue[str] = queue.Queue()
        self.columns: list[str] = []
        self.rows: list[list[Any]] = []
        self.summary: list[str] = []
        self.chart: dict[str, Any] | None = None
        self.error: str | None = None
        self.started = time.time()
        self.finished: float | None = None
        self.lock = threading.Lock()

    @property
    def active(self) -> bool:
        return self.status in ("running", "waiting")

    def check(self) -> None:
        if self.cancel.is_set():
            raise Cancelled

    def sleep(self, seconds: float) -> None:
        if self.cancel.wait(seconds):
            raise Cancelled

    def say(self, text: str) -> None:
        with self.lock:
            self.progress = text

    def ask(self, text: str, options: list[str]) -> str:
        """Show `text` with `options` on the page and wait for the user's pick."""
        with self.lock:
            self.question, self.status = {"text": text, "options": options}, "waiting"
        try:
            while True:
                self.check()
                try:
                    answer = self._answers.get(timeout=0.1)
                except queue.Empty:
                    continue
                if answer in options:
                    return answer
        finally:
            with self.lock:
                self.question = None
                if self.status == "waiting":
                    self.status = "running"

    def answer(self, value: str) -> None:
        self._answers.put(value)

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {"kind": self.kind, "title": self.title, "params": self.params, "status": self.status,
                    "progress": self.progress, "question": self.question, "columns": self.columns,
                    "rows": self.rows[-2000:], "row_count": len(self.rows), "summary": self.summary,
                    "chart": self.chart, "error": self.error, "started": self.started, "finished": self.finished}

    def csv(self) -> str:
        buf = io.StringIO()
        out = csv.writer(buf)
        out.writerow(self.columns)
        out.writerows(self.rows)
        return buf.getvalue()


# --- helpers ----------------------------------------------------------------------------------------------------------


def _num(params: dict[str, Any], key: str, default: float | None = None, lo: float = -math.inf,
         hi: float = math.inf) -> float:
    raw = params.get(key, default)
    try:
        value = float(raw)
    except (TypeError, ValueError) as e:
        raise Refused(f"{key}: not a number ({raw!r})") from e
    if not math.isfinite(value) or not lo <= value <= hi:
        raise Refused(f"{key} {value:g} outside {lo:g}..{hi:g}")
    return value


def _move_and_wait(bench: Bench, r: Routine, value: float, speed: float, acc: int, timeout: float | None = None) -> float:
    """Move to `value`, wait for the end; returns the seconds it took. Raises Refused on a refusal or a stop."""
    here = bench.value_now()
    distance = abs(value - here) if here is not None else 0.0
    if timeout is None:
        timeout = distance / speed * 1.5 + 5.0 + 255 / max(acc, 1) * 0.02
    t0 = time.monotonic()
    seq = bench.move_to(value, speed, acc, routine=True)
    status = bench.wait_motion_end(seq, timeout, r.cancel)
    r.check()
    if status is None:
        raise Refused(f"the move to {value:g} did not end within {timeout:.0f} s")
    if status == 3:
        raise Refused(f"the board stopped the move to {value:g} at a limit")
    return time.monotonic() - t0


def _settle_value(bench: Bench) -> float:
    counts = bench.fresh_counts()
    if counts is None:
        raise Refused("no encoder reply")
    value = bench.to_value(counts)
    if value is None:
        raise Refused("no reference")
    return value


def _stalled(bench: Bench) -> bool:
    reply = bench.read_now(Cmd.READ_STALL)
    return reply is not None and reply.status == 1


def _unit(bench: Bench) -> tuple[str, str]:
    return ("deg", "deg/s") if bench.mode == "joint" else ("motor deg", "RPM")


def _need_value(bench: Bench) -> float:
    value = bench.value_now()
    if value is None:
        raise Refused("no encoder reading: is the board answering?")
    return value


# --- repeat moves -----------------------------------------------------------------------------------------------------


def repeat_moves(bench: Bench, r: Routine, p: dict[str, Any]) -> None:
    """Back and forth between A and B, `cycles` times: the error at each end, the move times, stalls."""
    a, b = _num(p, "a"), _num(p, "b")
    cycles = int(_num(p, "cycles", 5, 1, 10_000))
    speed, acc = _num(p, "speed", None, 0), int(_num(p, "acc", 2, 0, 255))
    dwell = _num(p, "dwell", 0.5, 0, 60)
    bench.check(a, speed, acc)
    bench.check(b, speed, acc)
    unit, _ = _unit(bench)
    r.columns = ["cycle", "end", "target", "reached", f"error_{unit.replace(' ', '_')}", "move_s", "stalled"]
    r.say("to A")
    _need_value(bench)
    _move_and_wait(bench, r, a, speed, acc)
    r.sleep(dwell)
    errors: dict[str, list[float]] = {"A": [], "B": []}
    times: list[float] = []
    for cycle in range(1, cycles + 1):
        for end, target in (("B", b), ("A", a)):
            r.say(f"cycle {cycle}/{cycles}: to {end}")
            took = _move_and_wait(bench, r, target, speed, acc)
            r.sleep(dwell)
            reached = _settle_value(bench)
            stalled = _stalled(bench)
            error = reached - target
            errors[end].append(error)
            times.append(took)
            with r.lock:
                r.rows.append([cycle, end, round(target, 4), round(reached, 4), round(error, 4), round(took, 3), stalled])
                r.chart = {"kind": "lines", "x": "cycle", "y": f"error ({unit})",
                           "series": {f"at {k}": [[i + 1, e] for i, e in enumerate(v)] for k, v in errors.items()}}
            if stalled:
                raise Refused(f"the board reports a stall at cycle {cycle}, end {end}")
    resolution = 1 / bench.counts_per_unit()
    for end in ("A", "B"):
        e = errors[end]
        r.summary.append(f"at {end}: mean error {statistics.fmean(e):+.4f} {unit}, worst {max(e, key=abs):+.4f}, "
                         f"spread (sd) {statistics.pstdev(e):.4f}")
    r.summary.append(f"moves: mean {statistics.fmean(times):.2f} s, longest {max(times):.2f} s; one encoder count is "
                     f"{resolution:.4f} {unit}")


# --- step ---------------------------------------------------------------------------------------------------------------


def step_response(bench: Bench, r: Routine, p: dict[str, Any]) -> None:
    """A step of `step` from where it is, the encoder polled every 5 ms: rise, overshoot, settling, final error."""
    step = _num(p, "step", 5)
    if step == 0:
        raise Refused("a step of 0")
    speed, acc = _num(p, "speed", None, 0), int(_num(p, "acc", 2, 0, 255))
    duration = _num(p, "duration", 3, 0.5, 30)
    start = _need_value(bench)
    target = start + step
    bench.check(target, speed, acc)
    unit, _ = _unit(bench)
    r.columns = ["time_s", f"position_{unit.replace(' ', '_')}", "speed_rpm"]
    with bench.poll_profile((Cmd.READ_ENCODER,), (Cmd.READ_STATUS, Cmd.READ_SPEED), 0.005):
        r.sleep(0.2)
        seq0 = bench.sample_seq
        r.say(f"step {start:.3f} -> {target:.3f} {unit}")
        t_send = time.time()
        moved_seq = bench.move_to(target, speed, acc, routine=True)
        end = time.monotonic() + duration
        while time.monotonic() < end:
            r.sleep(0.05)
            r.say(f"recording {end - time.monotonic():.1f} s")
        bench.wait_motion_end(moved_seq, 1.0, r.cancel)
        samples = bench.samples_since(seq0)
    points = [(s[1] - t_send, bench.to_value(s[2]), s[3]) for s in samples if s[2] is not None]
    points = [(t, v, rpm) for t, v, rpm in points if v is not None]
    if len(points) < 10:
        raise Refused(f"only {len(points)} encoder samples came in {duration:g} s")
    with r.lock:
        r.rows = [[round(t, 4), round(v, 5), rpm] for t, v, rpm in points]
    sgn = 1 if step > 0 else -1
    frac = [(t, (v - start) / step) for t, v, _ in points]

    def first_at(level: float) -> float | None:
        return next((t for t, f in frac if t >= 0 and f >= level), None)

    t10, t90 = first_at(0.1), first_at(0.9)
    peak = max((v - target) * sgn for _, v, _ in points)
    band = max(0.02 * abs(step), 2 / bench.counts_per_unit())
    outside = [t for t, v, _ in points if abs(v - target) > band]
    settle = None if not outside else next((t for t, _, _ in points if t > outside[-1]), None)
    tail = [v for _, v, _ in points[-max(3, len(points) // 10):]]
    final_error = statistics.fmean(tail) - target
    r.summary += [
        (f"step {step:+g} {unit} at {speed:g} {_unit(bench)[1]}, acc {acc}; {len(points)} samples "
         f"({len(points) / duration:.0f} per second)"),
        "rise 10-90 %: " + ("n/a" if t10 is None or t90 is None else f"{t90 - t10:.3f} s (90 % at {t90:.3f} s)"),
        f"overshoot: {max(0.0, peak):.4f} {unit} ({max(0.0, peak) / abs(step) * 100:.1f} % of the step)",
        "settled (within " + f"{band:.4f} {unit}): " + ("n/a" if settle is None else f"{settle:.3f} s")
        + ("" if outside else " (never outside)"),
        f"final error: {final_error:+.4f} {unit}",
        "The board ramps the move itself (acc): this shows its profile and settling, not a raw step.",
    ]
    r.chart = {"kind": "lines", "x": "time (s)", "y": f"position ({unit})",
               "series": {"position": [[round(t, 4), v] for t, v, _ in points],
                          "target": [[points[0][0], target], [points[-1][0], target]]}}


# --- speed sweep --------------------------------------------------------------------------------------------------------


def speed_sweep(bench: Bench, r: Routine, p: dict[str, Any]) -> None:
    """Moves end to end at rising speeds: the measured speed, current and angle error over each move's middle half."""
    a, b = _num(p, "a"), _num(p, "b")
    acc = int(_num(p, "acc", 2, 0, 255))
    try:
        speeds = [float(s) for s in str(p.get("speeds", "")).replace(",", " ").split()]
    except ValueError as e:
        raise Refused("speeds: numbers separated by spaces or commas") from e
    if not speeds or len(speeds) > 40:
        raise Refused("speeds: 1 to 40 of them")
    for s in speeds:
        bench.check(a, s, acc)
        bench.check(b, s, acc)
    unit, speed_unit = _unit(bench)
    r.columns = ["speed_set", f"speed_measured_{speed_unit.replace('/', 'p')}", "current_mean_ma", "current_max_ma",
                 "angle_error_max_motor_deg", "move_s", "samples"]
    _need_value(bench)
    r.say("to the start")
    _move_and_wait(bench, r, a, speeds[0], acc)
    ends = (b, a)
    curves: dict[str, list[list[float]]] = {"current mean (mA)": [], "angle error max (motor deg x100)": []}
    with bench.poll_profile((Cmd.READ_ENCODER, Cmd.READ_SPEED, Cmd.READ_POWER, Cmd.READ_ANGLE_ERROR),
                            (Cmd.READ_STATUS, Cmd.READ_STALL), 0.01):
        for i, speed in enumerate(speeds):
            target = ends[i % 2]
            start = bench.value_now()
            r.say(f"{i + 1}/{len(speeds)}: {speed:g} {speed_unit} to {target:g}")
            seq0 = bench.sample_seq
            took = _move_and_wait(bench, r, target, speed, acc)
            lo_, hi_ = sorted((start, target))
            span = hi_ - lo_
            middle = [s for s in bench.samples_since(seq0) if s[2] is not None
                      and lo_ + span / 4 <= bench.to_value(s[2]) <= hi_ - span / 4]
            rpms = [abs(s[3]) for s in middle if s[3] is not None]
            amps = [s[5] for s in middle if s[5] is not None]
            errs = [abs(s[6]) * 360 / 51200 for s in middle if s[6] is not None]
            measured = statistics.fmean(rpms) if rpms else None
            if measured is not None and bench.mode == "joint":
                measured = bench.joint().rpm_to_dps(measured)
            row = [speed, None if measured is None else round(measured, 3),
                   round(statistics.fmean(amps)) if amps else None, max(amps) if amps else None,
                   round(max(errs), 4) if errs else None, round(took, 3), len(middle)]
            with r.lock:
                r.rows.append(row)
                if amps:
                    curves["current mean (mA)"].append([speed, statistics.fmean(amps)])
                if errs:
                    curves["angle error max (motor deg x100)"].append([speed, max(errs) * 100])
                r.chart = {"kind": "lines", "x": f"speed ({speed_unit})", "y": "", "series": curves}
            if _stalled(bench):
                raise Refused(f"the board reports a stall at {speed:g} {speed_unit}")
    if not any(row[2] is not None for row in r.rows):
        r.summary.append("No current readings: 0x38 is V1.0.9's - this board's firmware may not have it.")
    r.summary.append(f"{len(speeds)} speeds between {a:g} and {b:g} {unit}, acc {acc}; values over each move's "
                     "middle half (the ramps left out)")


# --- bring-up -----------------------------------------------------------------------------------------------------------


def firmware(bench: Bench, r: Routine, _p: dict[str, Any]) -> None:
    """What the board runs: 0x40, which newer reads it answers, its work mode and settings by 0x00."""
    r.columns = ["check", "result"]

    def row(check: str, result: str) -> None:
        with r.lock:
            r.rows.append([check, result])

    reply = bench.read_now(Cmd.READ_VERSION)
    v = None if reply is None else decode_version(reply)
    if v is None:
        raise Refused(f"no 0x40 reply from board {bench.can_id}: is it powered, on this id, on the bus?")
    board = HARDWARE_NAMES.get(v.hardware, str(v.hardware))
    row("0x40 firmware", f"{v.firmware_text} (bytes {v.firmware.hex(' ')})")
    row("0x40 board", board)
    row("0x40 calibrated", "yes" if v.calibrated else "NO - calibrate it unloaded (Settings, bench mode)")
    row("0x40 bootloader", f"0x{v.bootloader:02X} ({'CAN' if v.bootloader == 0x20 else 'RS485' if v.bootloader == 0x10 else '?'})")
    power = bench.read_now(Cmd.READ_POWER)
    p = None if power is None else decode_power(power)
    row("0x38 voltage + current (V1.0.9)", "no reply" if p is None else f"{p.millivolts / 1000:.2f} V, {p.milliamps} mA")
    home = bench.read_now(Cmd.READ_HOME_STATUS)
    row("0x3B home status", "no reply" if home is None else
        f"{len(home.data) - 1} byte(s) {home.data[1:].hex(' ')} ({'V1.0.7 or later' if len(home.data) >= 3 else 'before V1.0.7'})")
    settings = bench.read_settings()
    mode = settings.get("82")
    for code, label in (("82", "0x82 work mode"), ("83", "0x83 current, mA"), ("84", "0x84 microsteps"),
                        ("88", "0x88 stall protection"), ("8A", "0x8A CAN bitrate (0-3: 125K/250K/500K/1M)"),
                        ("8B", "0x8B CAN id"), ("8C", "0x8C response mode")):
        value = settings.get(code)
        row(f"{label} (0x00)", "no reply (0x00 is V1.0.6+)" if value is None else
            "not readable" if value["value"] is None else f"{value['value']} ({value['raw']})")
    mode_name = None if not mode or mode["value"] is None else WORK_MODES.get(mode["value"], str(mode["value"]))
    newer = p is not None
    r.summary.append(f"board {bench.can_id}: {board}, firmware {v.firmware_text}, "
                     f"{'calibrated' if v.calibrated else 'NOT calibrated'}; "
                     f"{'answers' if newer else 'does not answer'} V1.0.9's 0x38")
    if mode_name is not None and mode_name != "SR_vFOC":
        r.summary.append(f"Work mode {mode_name}: the motion commands need a bus mode - set SR_vFOC (Settings)")
    bench.add_finding(f"{bench.joint_config.name}: the board at CAN id {bench.can_id} runs firmware {v.firmware_text} "
                      f"({board}, bootloader 0x{v.bootloader:02X}){'' if newer else ', no 0x38 reply'}"
                      + ("" if mode_name is None else f", work mode {mode_name}"))


def direction(bench: Bench, r: Routine, p: dict[str, Any]) -> None:
    """Which way the encoder counts: a small step up in counts, then the user says which way the joint (or shaft) went."""
    joint_mode = bench.mode == "joint"
    amount = _num(p, "amount", 5 if joint_mode else 90, 0.1, 30 if joint_mode else 360)
    speed = _num(p, "speed", min(3.0, bench.speed_cap()) if joint_mode else 30, 0)
    name = bench.joint_config.name
    r.columns = ["step", "counts_before", "counts_after", "counted", "speed_rpm_sign"]
    if bench.readout.enabled is False:
        raise Refused("the motor is disabled: enable it first")
    room = (f"Is {name} at least {2 * amount:g} deg from both of its ends, and clear to turn {amount:g} deg either way?"
            if joint_mode else f"Is the motor free to turn {amount:g} deg (nothing on its shaft that can catch)?")
    if r.ask(room, ["Yes, go", "Cancel"]) != "Yes, go":
        raise Cancelled
    delta = round(amount * bench.counts_per_unit())
    c0 = bench.fresh_counts()
    if c0 is None:
        raise Refused("no encoder reply")
    seq0 = bench.sample_seq
    r.say(f"stepping {delta:+d} counts")
    seq = bench.move_counts_relative(delta, speed, 2, routine=True)
    if bench.wait_motion_end(seq, amount / speed * 2 + 6, r.cancel) is None:
        r.check()
        raise Refused("the step did not end")
    c1 = bench.fresh_counts()
    if c1 is None:
        raise Refused("no encoder reply after the step")
    rpms = [s[3] for s in bench.samples_since(seq0) if s[3]]
    rpm_sign = 0 if not rpms else (1 if max(rpms, key=abs) > 0 else -1)
    counted = c1 - c0
    with r.lock:
        r.rows.append(["up", c0, c1, counted, rpm_sign])
    if abs(counted) < abs(delta) / 4:
        raise Refused(f"the encoder moved {counted} counts for a {delta} count step: did the motor turn? (enabled, "
                      "work mode SR_vFOC, not stalled?)")
    if joint_mode:
        options = ["Positive: away from its home switch", "Negative: toward its home switch", "Could not tell"]
        text = (f"The encoder counted {'UP' if counted > 0 else 'DOWN'} by {abs(counted)} counts (about {amount:g} deg "
                f"of {name} by its ratio). Which way did {name} turn? (Every joint's home switch sits at its lower, "
                f"negative end: positive turns away from it.)")
    else:
        options = ["Clockwise", "Counter-clockwise", "Could not tell"]
        text = (f"The encoder counted {'UP' if counted > 0 else 'DOWN'} by {abs(counted)} counts. Looking at the "
                f"shaft's end (the motor's front), which way did it turn?")
    answer = r.ask(text, options)
    r.say("stepping back")
    seq = bench.move_counts_relative(-delta, speed, 2, routine=True)
    bench.wait_motion_end(seq, amount / speed * 2 + 6, r.cancel)
    r.check()
    if answer == "Could not tell":
        r.summary.append("No answer: run it again with a larger step.")
        return
    up = counted > 0
    speed_note = ("" if rpm_sign == 0 else
                  f"; 0x32 read {'positive' if rpm_sign > 0 else 'negative'} speeds while the encoder counted "
                  f"{'up' if up else 'down'}")
    if joint_mode:
        positive = answer.startswith("Positive")
        sign = 1 if positive == up else -1
        r.summary.append(f"{name}: the encoder counts {'up' if sign > 0 else 'down'} as the joint turns positive: "
                         f"sign {sign:+d} (config.JOINTS has {bench.joint_config.sign:+d}){speed_note}")
        cfg = bench.joint_config
        line = (f'JointConfig("{cfg.name}", 0x{bench.can_id:02X}, 1 / {1 / cfg.gear_ratio:g}, {sign:+d}, '
                f"({cfg.limits_deg[0]}, {cfg.limits_deg[1]}), {cfg.home_deg})")
        r.summary.append(f"config.py: {line}")
        bench.add_finding(f"{name}: sign {sign:+d} - the encoder counts {'up' if sign > 0 else 'down'} as the joint "
                          f"turns positive (direction check){speed_note}")
        if sign != bench.sign:
            bench.set_sign(sign, routine=True)
            r.summary.append(f"The dashboard now uses sign {sign:+d} for {name} (this session; the reference is cleared)")
    else:
        cw = answer == "Clockwise"
        counts_cw = cw == up
        r.summary.append(f"Seen from the shaft's end, the encoder counts {'up' if counts_cw else 'down'} turning "
                         f"clockwise{speed_note}")
        bench.add_finding(f"board {bench.can_id} (bench): the encoder counts {'up' if counts_cw else 'down'} as the "
                          f"shaft turns clockwise, seen from its end{speed_note}")


def switch_zone(bench: Bench, r: Routine, p: dict[str, Any]) -> None:
    """Slowly from A to B and back, IN_1 polled every 10 ms: where the home switch turns on and off each way."""
    a, b = _num(p, "a"), _num(p, "b")
    speed = _num(p, "speed", None, 0)
    acc = int(_num(p, "acc", 2, 0, 255))
    bench.check(a, speed, acc)
    bench.check(b, speed, acc)
    unit, _ = _unit(bench)
    r.columns = ["leg", "edge", "position", "in1_before", "in1_after"]
    _need_value(bench)
    r.say("to A")
    _move_and_wait(bench, r, a, speed, acc)
    edges: list[tuple[str, str, float]] = []
    with bench.poll_profile((Cmd.READ_ENCODER, Cmd.READ_IO), (Cmd.READ_STATUS, Cmd.READ_SPEED, Cmd.READ_STALL), 0.01):
        r.sleep(0.2)
        for leg, target in (("A->B", b), ("B->A", a)):
            r.say(f"{leg} at {speed:g}")
            seq0 = bench.sample_seq
            _move_and_wait(bench, r, target, speed, acc)
            rows = [s for s in bench.samples_since(seq0) if s[2] is not None and s[7] is not None]
            for before, after in itertools.pairwise(rows):
                if (before[7] & 1) != (after[7] & 1):
                    where = (bench.to_value(before[2]) + bench.to_value(after[2])) / 2
                    edge = "on (IN_1 low: magnet seen)" if after[7] & 1 == 0 else "off (IN_1 high)"
                    edges.append((leg, edge, where))
                    with r.lock:
                        r.rows.append([leg, edge, round(where, 4), before[7] & 1, after[7] & 1])
    if not edges:
        r.summary.append(f"IN_1 never changed between {a:g} and {b:g} {unit}: the switch's zone is not in this span, "
                         "or the switch is not wired to IN_1 (pin 3 of the 5-pin plug)")
        return
    name = bench.joint_config.name
    legs = []
    for leg in ("A->B", "B->A"):
        found = [f"{e.split(' ')[0]} at {w:.2f}" for lg, e, w in edges if lg == leg]
        legs.append(f"{leg}: {', '.join(found) if found else 'no change'}")
    r.summary.append(f"{'; '.join(legs)} {unit}")
    for leg in ("A->B", "B->A"):
        on = [w for lg, e, w in edges if lg == leg and e.startswith("on")]
        off = [w for lg, e, w in edges if lg == leg and e.startswith("off")]
        if on and off:
            r.summary.append(f"{leg}: the zone runs {min(on + off):.2f} .. {max(on + off):.2f} {unit} "
                             f"(half width {(max(on + off) - min(on + off)) / 2:.2f})")
    if len(edges) <= 2 and all(sum(1 for lg, _, _ in edges if lg == leg) <= 1 for leg in ("A->B", "B->A")):
        r.summary.append("One edge each way: the zone runs on past the span's end (as each joint's does past its "
                         "lower limit) - only its near edge is in reach")
    if bench.mode == "joint" and bench.zero_deg is not None:   # joint angles: comparable with home_deg
        toward_minus = [w for lg, e, w in edges if e.startswith("on") and
                        ((lg == "A->B") == (b < a))]
        if toward_minus:
            home = toward_minus[0]
            r.summary.append(f"turning toward -, it turns on at {home:.2f} deg - where homing stops; config's "
                             f"home_deg is {bench.joint_config.home_deg:g} ({home - bench.joint_config.home_deg:+.2f})")
        bench.add_finding(f"{name}: home switch {r.summary[0]} at {speed:g} deg/s"
                          + (f"; {r.summary[-1]}" if r.summary[-1].startswith("turning toward") else ""))


def home(bench: Bench, r: Routine, p: dict[str, Any]) -> None:
    """0x90 + 0x91: the board searches its switch; in joint mode the reference follows (the edge is home_deg)."""
    trigger = int(_num(p, "trigger", 0, 0, 1))
    direction_ = int(_num(p, "direction", 0, 0, 1))
    rpm = int(_num(p, "rpm", 10, 1, 3000))
    end_limit = bool(int(_num(p, "end_limit", 0, 0, 1)))
    hm_mode = p.get("hm_mode")
    hm_mode = None if hm_mode in (None, "", "none") else int(_num(p, "hm_mode", 0, 0, 2))
    if bench.mode == "joint":
        dps = bench.joint().rpm_to_dps(rpm)
        if dps > bench.speed_cap() + 1e-9:
            raise Refused(f"{rpm} RPM is {dps:.1f} deg/s at {bench.joint_config.name}, over the {bench.speed_cap():g} "
                          "deg/s cap")
        lo, hi = bench.joint_config.limits_deg
        timeout = (hi - lo) / dps * 1.3 + 10
    else:
        timeout = _num(p, "timeout", 60, 5, 600)
    r.columns = ["step", "result"]
    r.say(f"homing at {rpm} RPM, direction {'CCW' if direction_ else 'CW'} - press STOP if it heads the wrong way")
    seq = bench.send_home(trigger, direction_, rpm, end_limit, hm_mode)
    try:
        deadline = time.monotonic() + timeout
        result = None
        seen_going = False   # 0x3B's homing byte read 0 (going) or 0xF1 read 5 (homing): a later 1/2 is this run's
        while time.monotonic() < deadline and result is None:
            r.check()
            for _, reply in bench.worker.events_since(seq):
                if reply.can_id == bench.can_id and reply.code == Cmd.GO_HOME and reply.status in (0, 2):
                    result = reply.status
            hs = bench.read_now(Cmd.READ_HOME_STATUS)
            seen_going = seen_going or bench.readout.status == 5 or (hs is not None and len(hs.data) >= 2
                                                                     and hs.data[-1] == 0)
            if result is None and seen_going and hs is not None and len(hs.data) >= 2 and hs.data[-1] in (1, 2):
                result = 2 if hs.data[-1] == 1 else 0
            r.sleep(0.1)
    finally:
        bench.set_motion(None)
    if result is None:
        raise Refused(f"homing did not finish within {timeout:.0f} s")
    with r.lock:
        r.rows.append(["0x91", "complete" if result == 2 else "failed"])
    if result != 2:
        raise Refused("the board reports homing failed")
    bench.homed()
    if bench.mode == "joint":
        r.summary.append(f"Homed: the encoder's zero is the switch's edge, {bench.joint_config.home_deg:g} deg "
                         f"(config's home_deg). Joint mode is referenced.")
    else:
        r.summary.append("Homed: the encoder's zero is the switch's edge.")


def calibrate(bench: Bench, r: Routine, _p: dict[str, Any]) -> None:
    """0x80, bench mode only: the motor turns by itself and must be unloaded."""
    if bench.mode != "bench":
        raise Refused("calibrate only in bench mode, with the motor off the arm: it turns by itself, unloaded")
    if r.ask("Calibration turns the motor by itself. Is it unloaded (no belt, gear or arm on its shaft)?",
             ["Yes, calibrate", "Cancel"]) != "Yes, calibrate":
        raise Cancelled
    bench.set_motion("calibrate")
    try:
        seq = bench.worker.event_seq
        first = bench.worker.request(bench.can_id, bytes([Cmd.CALIBRATE, 0x00]), timeout=0.5)
        if first is None:
            raise Refused("no reply to 0x80")
        status = first.status
        deadline = time.monotonic() + 40
        while status == 0 and time.monotonic() < deadline:
            r.check()
            r.say("calibrating ...")
            reply = bench.worker.wait_event(seq, lambda x: x.can_id == bench.can_id and x.code == Cmd.CALIBRATE
                                            and x.status != 0, 0.2, r.cancel)
            if reply is not None:
                status = reply.status
    finally:
        bench.set_motion(None)
    r.columns = ["step", "result"]
    r.rows.append(["0x80", {0: "still calibrating after 40 s", 1: "success", 2: "failed"}.get(status, str(status))])
    if status != 1:
        raise Refused("calibration did not succeed")
    r.summary.append("Calibrated.")


ROUTINES: dict[str, tuple[str, Callable[[Bench, Routine, dict[str, Any]], None]]] = {
    "repeat": ("Repeat moves", repeat_moves),
    "step": ("Step response", step_response),
    "sweep": ("Speed sweep", speed_sweep),
    "firmware": ("Firmware check", firmware),
    "direction": ("Direction check", direction),
    "switch": ("Home switch zone", switch_zone),
    "home": ("Homing", home),
    "calibrate": ("Calibration", calibrate),
}


def start(bench: Bench, kind: str, params: dict[str, Any]) -> Routine:
    """Start a routine on its own thread (one at a time)."""
    if kind not in ROUTINES:
        raise Refused(f"no test {kind!r}")
    if bench.routine is not None and bench.routine.active:
        raise Refused(f"{bench.routine.title} is running: cancel it first")
    if bench.motion is not None:
        raise Refused(f"the motor is busy ({bench.motion}): stop it first")
    title, fn = ROUTINES[kind]
    routine = Routine(kind, title, dict(params))
    bench.routine = routine

    def run() -> None:
        status, error = "done", None
        try:
            fn(bench, routine, params)
        except Cancelled:
            status = "cancelled"
        except Refused as e:
            status, error = "failed", str(e)
        except Exception as e:  # noqa: BLE001 - shown on the page; the motor is stopped below
            status, error = "failed", f"internal error: {e!r}"
        if status != "done":
            bench.stop()   # before the routine shows as ended: once it is, its stop has been queued
        with routine.lock:
            routine.error, routine.finished, routine.progress = error, time.time(), status
            routine.status = status
        bench.notice(f"{title}: {status}" + (f" - {error}" if error else ""), "info" if status == "done" else "warn")

    threading.Thread(target=run, name=f"dashboard-{kind}", daemon=True).start()
    return routine


__all__ = ["MOTOR_DEG_PER_COUNT", "ROUTINES", "Routine", "start"]
