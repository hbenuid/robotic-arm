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


class BusWorker:
    """The thread that owns `can`. Every public method may be called from any thread.

    on_reply(reply, is_poll) and on_tick(now) are called on the worker thread: keep them short, never block in them.
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
        # polling: worker-thread state, set through set_poll()
        self._poll_lock = threading.Lock()
        self._poll_id: int | None = None
        self._fast: tuple[int, ...] = ()
        self._slow: tuple[int, ...] = ()
        self._fast_s = 0.02
        self._slow_s = 0.25
        self._next_fast = 0.0
        self._next_slow = 0.0
        self._slow_index = 0
        self._poll_reset = False
        self._inflight: dict[int, float] = {}    # worker thread only, like _misses and _skipped
        self._misses: collections.Counter[int] = collections.Counter()
        self._skipped: dict[int, float] = {}
        self._skipped_view: tuple[int, ...] = ()  # _skipped's keys, for the other threads
        self.last_reply: dict[int, float] = {}   # can_id -> time.monotonic() of its last checked reply
        self.bus_error: str | None = None
        # every key there from the start: the page's threads copy it while the worker counts
        self.counts = collections.Counter(tx=0, rx=0, missed=0, bad=0, bus_error=0)
        self._tick_error_at = 0.0
        self.on_reply: Callable[[Reply, bool], None] = lambda _reply, _poll: None
        self.on_tick: Callable[[float], None] = lambda _now: None
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

    def set_poll(self, can_id: int | None, fast: Iterable[int], slow: Iterable[int],
                 fast_s: float = 0.02, slow_s: float = 0.25) -> None:
        """Poll `fast` every `fast_s` and `slow` (one at a time, round robin) every `slow_s / len(slow)` on `can_id`."""
        with self._poll_lock:
            self._poll_reset = self._poll_reset or can_id != self._poll_id   # the worker clears its poll state
            self._poll_id, self._fast, self._slow = can_id, tuple(fast), tuple(slow)
            self._fast_s, self._slow_s = fast_s, slow_s

    @property
    def poll_id(self) -> int | None:
        return self._poll_id

    def skipped_reads(self) -> list[int]:
        """The poll reads skipped as unanswered (the board's firmware may not have them)."""
        return list(self._skipped_view)

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
            self._skipped_view = tuple(sorted(self._skipped))
            try:
                self.on_tick(time.monotonic())
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
        with self._poll_lock:
            can_id, fast, slow = self._poll_id, self._fast, self._slow
            fast_s, slow_s = self._fast_s, self._slow_s
            reset, self._poll_reset = self._poll_reset, False
        if reset:
            self._inflight.clear()
            self._misses.clear()
            self._skipped.clear()
        if can_id is None:
            return
        codes: list[int] = []
        if fast and now >= self._next_fast:
            self._next_fast = now + fast_s
            codes += fast
        if slow and now >= self._next_slow:
            self._next_slow = now + slow_s / len(slow)
            self._slow_index = (self._slow_index + 1) % len(slow)
            codes.append(slow[self._slow_index])
        for code in codes:
            if code in self._inflight:
                continue
            if code in self._skipped and now < self._skipped[code]:
                continue
            if self._transmit(can_id, bytes([code]), now, None, True):
                self._inflight[code] = now + POLL_REPLY_S

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
        is_poll = frame.can_id == self._poll_id and reply.code in self._inflight
        if is_poll:
            del self._inflight[reply.code]
            self._misses[reply.code] = 0
            self._skipped.pop(reply.code, None)
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
        try:
            self.on_reply(reply, is_poll)
        except Exception as e:  # noqa: BLE001 - as on_tick: report, keep the bus thread alive
            self.note(f"internal error handling a reply: {e!r}")

    def _expire(self, now: float) -> None:
        for waiter in [w for w in self._waiters if now > w.deadline]:
            self._waiters.remove(waiter)
            waiter.future.set_result(None)
        answered = self._poll_id is not None and now - self.last_reply.get(self._poll_id, -1e9) < 1.0
        for code, deadline in list(self._inflight.items()):
            if now > deadline:
                self._inflight.pop(code, None)
                self.counts["missed"] += 1
                self._misses[code] += 1
                if answered and self._misses[code] >= UNSUPPORTED_AFTER and code not in self._skipped:
                    self._skipped[code] = now + UNSUPPORTED_RETRY_S
                    self.note(f"0x{code:02X} ({NAMES.get(code, '?')}) unanswered {UNSUPPORTED_AFTER} times while the "
                              f"board answers other reads: skipped (its firmware may not have it), retried every "
                              f"{UNSUPPORTED_RETRY_S:.0f} s")
                elif code in self._skipped:
                    self._skipped[code] = now + UNSUPPORTED_RETRY_S
