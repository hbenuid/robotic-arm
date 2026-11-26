"""The dashboard's bus thread: the one thread that touches the CAN bus.

CANInterface is not thread-safe, and the page, the test routines and the safety checks all need the bus, so all of it
goes through one worker thread. It sends what the other threads queue (a stop jumps the queue), polls the selected
board's readouts, hands each reply to the waiter that asked for it, keeps every other reply as an event (a move's
second reply, when it ends), and logs every frame. It never blocks for more than a few milliseconds, so a stop goes
out at once.
"""

from __future__ import annotations

import collections
import threading
import time
from collections.abc import Callable, Iterable
from concurrent.futures import Future
from dataclasses import dataclass

from robotic_arm.can_interface import CANInterface, Frame
from robotic_arm.motor_driver import Checksum, Cmd, MotorDriver, Reply

RECEIVE_S = 0.002          # the worker's longest wait on the bus: how late a queued stop can go out
POLL_REPLY_S = 0.1         # a poll read not answered in this long counts as missed
UNSUPPORTED_AFTER = 5      # misses in a row (while the board answers other reads) before a read is skipped
UNSUPPORTED_RETRY_S = 5.0  # a skipped read is tried again after this long
LOG_MAX = 4000
EVENTS_MAX = 2000

# Read codes: a reply answers every waiter for that code, since any fresh value will do.
READ_CODES = frozenset({0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x38, 0x39, 0x3A, 0x3B, 0x3E, 0x40, 0xF1})

NAMES = {c.value: c.name.lower().replace("_", " ") for c in Cmd} | {
    0x84: "set microsteps", 0x89: "set mplyer", 0x8A: "set can bitrate", 0x8B: "set can id",
    0x8C: "set response mode", 0x8D: "set group id", 0x9A: "single-turn zero", 0x9D: "set position protection",
    0x9E: "set limit remap", 0x94: "set limit homing",
}


def describe(direction: str, data: bytes) -> str:
    """A log line's meaning for `data` (code first, no check byte), as sent ('tx') or received ('rx')."""
    if not data:
        return ""
    code, rest = data[0], data[1:]
    name = NAMES.get(code, f"0x{code:02X}")
    if direction == "tx" and code in (Cmd.MOVE_TO_AXIS, Cmd.MOVE_RELATIVE_AXIS) and len(rest) == 6:
        speed, acc = int.from_bytes(rest[0:2], "big"), rest[2]
        value = int.from_bytes(rest[3:6], "big", signed=True)
        if speed == 0:
            return f"{name}: stop ({'at once' if acc == 0 else f'ramp {acc}'})"
        return f"{name}: {'to' if code == Cmd.MOVE_TO_AXIS else 'by'} {value} counts at {speed} RPM, acc {acc}"
    if direction == "tx" and code == Cmd.SPEED and len(rest) >= 3:
        speed = ((rest[0] & 0x0F) << 8) | rest[1]
        if speed == 0:
            return f"{name}: stop ({'at once' if rest[2] == 0 else f'ramp {rest[2]}'})"
        return f"{name}: {'CW' if rest[0] & 0x80 else 'CCW'} at {speed} RPM, acc {rest[2]}"
    if direction == "tx":
        return name + (f" {rest.hex(' ')}" if rest else "")
    return f"reply {code:02X} ({name}): {rest.hex(' ') or '(empty)'}"


@dataclass(frozen=True)
class LogEntry:
    seq: int
    t: float          # time.time()
    direction: str    # "tx", "rx", "note"
    can_id: int | None
    hex: str          # the frame's bytes, check byte included
    text: str


class _Waiter:
    __slots__ = ("accept", "can_id", "code", "deadline", "future")

    def __init__(self, can_id: int, code: int, deadline: float, accept: Callable[[Reply], bool] | None):
        self.can_id, self.code, self.deadline, self.accept = can_id, code, deadline, accept
        self.future: Future[Reply | None] = Future()


class _PollPlan:
    """One board's polling: the reads, their pace, what is in flight, what goes unanswered."""

    __slots__ = ("fast", "fast_s", "inflight", "misses", "next_fast", "next_slow", "skipped", "slow", "slow_index",
                 "slow_s")

    def __init__(self, fast: tuple[int, ...], slow: tuple[int, ...], fast_s: float, slow_s: float):
        self.fast, self.slow, self.fast_s, self.slow_s = fast, slow, fast_s, slow_s
        self.next_fast = self.next_slow = 0.0
        self.slow_index = 0
        self.inflight: dict[int, float] = {}
        self.misses: collections.Counter[int] = collections.Counter()
        self.skipped: dict[int, float] = {}


class BusWorker:
    """The thread that owns `can`. Every public method may be called from any thread.

    It polls every board given to set_poll(). The listeners' on_reply(reply, is_poll) and on_tick(now) are called on
    the worker thread, for every board's replies and every loop: keep them short, never block in them.
    """

    def __init__(self, can: CANInterface, checksum: Checksum = Checksum.SUM8):
        self._can = can
        self._checksum = checksum
        self._drivers: dict[int, MotorDriver] = {}
        self._out: collections.deque[tuple[int, bytes, _Waiter | None, bool]] = collections.deque()
        self._out_lock = threading.Lock()
        self._waiters: list[_Waiter] = []
        self._cond = threading.Condition()
        self._events: collections.deque[tuple[int, Reply]] = collections.deque(maxlen=EVENTS_MAX)
        self._event_seq = 0
        self._log: collections.deque[LogEntry] = collections.deque(maxlen=LOG_MAX)
        self._log_seq = 0
        self._log_lock = threading.Lock()
        self.log_polls = False
        self._poll_lock = threading.Lock()
        self._plans: dict[int, _PollPlan] = {}   # can_id -> its polling, under _poll_lock
        self.last_reply: dict[int, float] = {}   # can_id -> time.monotonic() of its last checked reply
        self.bus_error: str | None = None
        # every key there from the start: the page's threads copy it while the worker counts
        self.counts = collections.Counter(tx=0, rx=0, missed=0, bad=0, bus_error=0)
        self._tick_error_at = 0.0
        self._listeners: tuple[tuple[Callable[[Reply, bool], None], Callable[[float], None]], ...] = ()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="dashboard-bus", daemon=True)

    # --- from any thread ----------------------------------------------------------------------------------------

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def driver(self, can_id: int) -> MotorDriver:
        with self._out_lock:
            if can_id not in self._drivers:
                self._drivers[can_id] = MotorDriver(self._can, can_id, self._checksum)
            return self._drivers[can_id]

    def submit(self, can_id: int, data: bytes, *, code: int | None = None, timeout: float = 0.15,
               accept: Callable[[Reply], bool] | None = None, priority: bool = False,
               wait: bool = True) -> Future[Reply | None]:
        """Queue `data` (no check byte) for `can_id`. With `wait`, the future resolves to the board's reply with
        `code` (default: the request's own) or None after `timeout`; without, it resolves None once sent."""
        self.driver(can_id).encode(data)   # a bad length raises here, in the caller's thread
        waiter = _Waiter(can_id, data[0] if code is None else code, timeout, accept) if wait else None
        with self._out_lock:
            item = (can_id, bytes(data), waiter, wait)
            if priority:
                self._out.appendleft(item)
            else:
                self._out.append(item)
        if waiter is None:
            done: Future[Reply | None] = Future()
            done.set_result(None)
            return done
        return waiter.future

    def request(self, can_id: int, data: bytes, *, code: int | None = None, timeout: float = 0.15,
                accept: Callable[[Reply], bool] | None = None, priority: bool = False) -> Reply | None:
        """submit() and wait for the reply (None: none came)."""
        future = self.submit(can_id, data, code=code, timeout=timeout, accept=accept, priority=priority)
        try:
            return future.result(timeout + 2.0)
        except TimeoutError:
            return None

    def send(self, can_id: int, data: bytes, *, priority: bool = False) -> None:
        self.submit(can_id, data, wait=False, priority=priority)

    def purge(self, can_id: int, codes: Iterable[int]) -> int:
        """Drop the frames with these codes still queued for `can_id` (a stop must not be followed by a move queued
        before it); their waiters get None. Returns how many."""
        codes = set(codes)
        with self._out_lock:
            keep, dropped = collections.deque(), []
            for item in self._out:
                (dropped if item[0] == can_id and item[1][0] in codes else keep).append(item)
            self._out = keep
        for _, _, waiter, _ in dropped:
            if waiter is not None:
                waiter.future.set_result(None)
        return len(dropped)

    def add_listener(self, on_reply: Callable[[Reply, bool], None], on_tick: Callable[[float], None]) -> None:
        with self._poll_lock:
            self._listeners = (*self._listeners, (on_reply, on_tick))

    def remove_listener(self, on_reply: Callable[[Reply, bool], None]) -> None:
        with self._poll_lock:
            self._listeners = tuple(pair for pair in self._listeners if pair[0] != on_reply)

    def set_poll(self, can_id: int, fast: Iterable[int], slow: Iterable[int],
                 fast_s: float = 0.02, slow_s: float = 0.25) -> None:
        """Poll `fast` every `fast_s` and `slow` (one at a time, round robin) every `slow_s / len(slow)` on `can_id`;
        a board already polled keeps what it has in flight."""
        with self._poll_lock:
            plan = self._plans.get(can_id)
            if plan is None:
                self._plans[can_id] = _PollPlan(tuple(fast), tuple(slow), fast_s, slow_s)
            else:
                plan.fast, plan.slow, plan.fast_s, plan.slow_s = tuple(fast), tuple(slow), fast_s, slow_s
                plan.slow_index %= max(1, len(plan.slow))

    def remove_poll(self, can_id: int) -> None:
        with self._poll_lock:
            self._plans.pop(can_id, None)

    @property
    def poll_ids(self) -> list[int]:
        with self._poll_lock:
            return list(self._plans)

    def skipped_reads(self, can_id: int) -> list[int]:
        """`can_id`'s poll reads skipped as unanswered (the board's firmware may not have them)."""
        with self._poll_lock:
            plan = self._plans.get(can_id)
            return [] if plan is None else sorted(plan.skipped)

    @property
    def event_seq(self) -> int:
        return self._event_seq

    def events_since(self, seq: int) -> list[tuple[int, Reply]]:
        with self._cond:
            return [(s, r) for s, r in self._events if s > seq]

    def wait_event(self, after: int, match: Callable[[Reply], bool], timeout: float,
                   cancel: threading.Event | None = None) -> Reply | None:
        """The first event after seq `after` that `match` accepts; None on timeout or `cancel`."""
        deadline = time.monotonic() + timeout
        with self._cond:
            while True:
                for s, reply in self._events:
                    if s > after and match(reply):
                        return reply
                remaining = deadline - time.monotonic()
                if remaining <= 0 or (cancel is not None and cancel.is_set()):
                    return None
                self._cond.wait(min(remaining, 0.05))

    def note(self, text: str) -> None:
        """A line in the log that is not a frame."""
        self._add_log("note", None, b"", text)

    def log_since(self, seq: int) -> list[LogEntry]:
        with self._log_lock:
            return [e for e in self._log if e.seq > seq]

    # --- the worker thread ----------------------------------------------------------------------------------------

    def _add_log(self, direction: str, can_id: int | None, raw: bytes, text: str) -> None:
        with self._log_lock:
            self._log_seq += 1
            self._log.append(LogEntry(self._log_seq, time.time(), direction, can_id, raw.hex(" "), text))

    def _run(self) -> None:
        while not self._stop.is_set():
            now = time.monotonic()
            self._flush(now)
            self._poll(now)
            self._receive()
            self._expire(time.monotonic())
            for _, on_tick in self._listeners:
                try:
                    on_tick(time.monotonic())
                except Exception as e:  # noqa: BLE001 - a bug in a check must not kill the bus thread (and the stops)
                    if time.monotonic() - self._tick_error_at > 1.0:
                        self._tick_error_at = time.monotonic()
                        self.note(f"internal error in the safety tick: {e!r}")
        for waiter in self._waiters:
            waiter.future.set_result(None)

    def _flush(self, now: float) -> None:
        while True:
            with self._out_lock:
                if not self._out:
                    return
                can_id, data, waiter, _ = self._out.popleft()
            self._transmit(can_id, data, now, waiter, False)

    def _transmit(self, can_id: int, data: bytes, now: float, waiter: _Waiter | None, is_poll: bool) -> bool:
        driver = self.driver(can_id)
        try:
            driver.send(data)
        except Exception as e:  # noqa: BLE001 - python-can, serial or OS: report it, keep the thread alive
            self.bus_error = f"{type(e).__name__}: {e}"
            self.counts["bus_error"] += 1
            self._add_log("note", can_id, b"", f"send failed: {self.bus_error}")
            if waiter is not None:
                waiter.future.set_result(None)
            return False
        self.bus_error = None
        self.counts["tx"] += 1
        if not is_poll or self.log_polls:
            self._add_log("tx", can_id, driver.encode(data), describe("tx", data))
        if waiter is not None:
            waiter.deadline = now + waiter.deadline   # submit() stored the timeout; it runs from the send
            self._waiters.append(waiter)
        return True

    def _poll(self, now: float) -> None:
        due: list[tuple[int, int]] = []
        with self._poll_lock:
            for can_id, plan in self._plans.items():
                codes: list[int] = []
                if plan.fast and now >= plan.next_fast:
                    plan.next_fast = now + plan.fast_s
                    codes += plan.fast
                if plan.slow and now >= plan.next_slow:
                    plan.next_slow = now + plan.slow_s / len(plan.slow)
                    plan.slow_index = (plan.slow_index + 1) % len(plan.slow)
                    codes.append(plan.slow[plan.slow_index])
                for code in codes:
                    if code in plan.inflight or (code in plan.skipped and now < plan.skipped[code]):
                        continue
                    plan.inflight[code] = now + POLL_REPLY_S
                    due.append((can_id, code))
        for can_id, code in due:   # sent outside the lock; a failed send is no longer in flight
            if not self._transmit(can_id, bytes([code]), now, None, True):
                with self._poll_lock:
                    plan = self._plans.get(can_id)
                    if plan is not None:
                        plan.inflight.pop(code, None)

    def _receive(self) -> None:
        frame = self._can.receive(timeout=RECEIVE_S)
        while frame is not None:
            self._dispatch(frame)
            frame = self._can.receive(timeout=0.0)

    def _dispatch(self, frame: Frame) -> None:
        self.counts["rx"] += 1
        driver = self._drivers.get(frame.can_id)
        reply = None if driver is None else driver.parse(frame)
        if reply is None:
            self.counts["bad"] += 1
            why = "no board selected at this id" if driver is None else "bad check byte or too short"
            self._add_log("rx", frame.can_id, frame.data, f"unparsed frame ({why})")
            return
        self.last_reply[frame.can_id] = time.monotonic()
        with self._poll_lock:
            plan = self._plans.get(frame.can_id)
            is_poll = plan is not None and reply.code in plan.inflight
            if is_poll:
                del plan.inflight[reply.code]
                plan.misses[reply.code] = 0
                plan.skipped.pop(reply.code, None)
        for waiter in list(self._waiters):
            if waiter.can_id == reply.can_id and waiter.code == reply.code and (waiter.accept is None or waiter.accept(reply)):
                self._waiters.remove(waiter)
                waiter.future.set_result(reply)
                if reply.code not in READ_CODES:
                    break
        if not is_poll:
            with self._cond:
                self._event_seq += 1
                self._events.append((self._event_seq, reply))
                self._cond.notify_all()
        if not is_poll or self.log_polls:
            self._add_log("rx", frame.can_id, frame.data, describe("rx", reply.data))
        for on_reply, _ in self._listeners:
            try:
                on_reply(reply, is_poll)
            except Exception as e:  # noqa: BLE001 - as on_tick: report, keep the bus thread alive
                self.note(f"internal error handling a reply: {e!r}")

    def _expire(self, now: float) -> None:
        for waiter in [w for w in self._waiters if now > w.deadline]:
            self._waiters.remove(waiter)
            waiter.future.set_result(None)
        notes: list[tuple[int, int]] = []
        with self._poll_lock:
            for can_id, plan in self._plans.items():
                answered = now - self.last_reply.get(can_id, -1e9) < 1.0
                for code, deadline in list(plan.inflight.items()):
                    if now <= deadline:
                        continue
                    del plan.inflight[code]
                    self.counts["missed"] += 1
                    plan.misses[code] += 1
                    if answered and plan.misses[code] >= UNSUPPORTED_AFTER and code not in plan.skipped:
                        plan.skipped[code] = now + UNSUPPORTED_RETRY_S
                        notes.append((can_id, code))
                    elif code in plan.skipped:
                        plan.skipped[code] = now + UNSUPPORTED_RETRY_S
        for can_id, code in notes:
            self.note(f"board {can_id}: 0x{code:02X} ({NAMES.get(code, '?')}) unanswered {UNSUPPORTED_AFTER} times while "
                      f"it answers other reads: skipped (its firmware may not have it), retried every "
                      f"{UNSUPPORTED_RETRY_S:.0f} s")
