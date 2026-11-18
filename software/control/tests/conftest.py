"""Shared fixtures: every test gets its own python-can virtual channel - nothing here reaches hardware."""

from __future__ import annotations

import itertools
import threading
from collections.abc import Iterator

import pytest

from robotic_arm.can_interface import CANInterface
from robotic_arm.fake import FakeBoards

REPLY_S = 1.0     # a reply the test expects: generous, it comes in milliseconds
SILENT_S = 0.05   # a reply the test expects NOT to come

_channels = itertools.count()


@pytest.fixture
def channel(request: pytest.FixtureRequest) -> str:
    return f"{request.node.name}-{next(_channels)}"


@pytest.fixture
def can(channel: str) -> Iterator[CANInterface]:
    with CANInterface(channel, "virtual", bitrate=None) as bus:
        yield bus


@pytest.fixture
def peer(channel: str) -> Iterator[CANInterface]:
    """A second bus on the same channel: the far end, for tests that write a board's frames by hand."""
    with CANInterface(channel, "virtual", bitrate=None) as bus:
        yield bus


@pytest.fixture
def fake(channel: str) -> Iterator[FakeBoards]:
    with FakeBoards(channel, range(1, 7)) as boards:
        yield boards


def answer_once(peer: CANInterface, raw: bytes, can_id: int = 1, reply_id: int | None = None) -> threading.Thread:
    """A thread that waits for one frame to `can_id` and answers it with `raw` (check byte included) verbatim, from
    `reply_id` (default: `can_id`)."""

    def run() -> None:
        if peer.receive(can_id, timeout=2.0) is not None:
            peer.send(can_id if reply_id is None else reply_id, raw)

    thread = threading.Thread(target=run)
    thread.start()
    return thread
