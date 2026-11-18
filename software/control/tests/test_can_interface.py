"""robotic_arm.can_interface on python-can's virtual bus."""

from __future__ import annotations

import threading
import time

import can as pycan
import pytest
from conftest import REPLY_S, SILENT_S

from robotic_arm.can_interface import PENDING_MAX, CANInterface, Frame


def test_send_and_receive_carry_the_id_the_data_and_a_timestamp(can, peer):
    before = time.time()
    can.send(0x01, bytes([1, 2, 3]))
    frame = peer.receive(0x01, REPLY_S)
    assert isinstance(frame, Frame)
    assert (frame.can_id, frame.data) == (0x01, bytes([1, 2, 3]))
    assert before <= frame.timestamp <= time.time()   # the virtual bus stamps with time.time()


def test_receive_without_an_id_takes_any_frame(can, peer):
    can.send(0x05, b"\xaa")
    assert peer.receive(timeout=REPLY_S)[:2] == (0x05, b"\xaa")


def test_a_round_trip_through_an_echoing_peer(can, peer):
    def echo() -> None:
        frame = peer.receive(timeout=2.0)
        if frame is not None:
            peer.send(frame.can_id, b"\x30\x40")

    thread = threading.Thread(target=echo)
    thread.start()
    can.send(0x01, b"\x10\x20")
    frame = can.receive(0x01, REPLY_S)
    thread.join(timeout=3.0)
    assert frame.data == b"\x30\x40"


def test_receive_times_out_with_none(can):
    start = time.monotonic()
    assert can.receive(0x01, SILENT_S) is None
    assert time.monotonic() - start >= SILENT_S


def test_a_frame_passed_over_is_kept_for_a_later_receive(can, peer):
    can.send(0x99, b"\xff")
    can.send(0x01, b"\xab")
    assert peer.receive(0x01, REPLY_S).data == b"\xab"
    assert peer.receive(timeout=SILENT_S)[:2] == (0x99, b"\xff")


def test_match_picks_by_content(can, peer):
    for byte in (1, 2, 3):
        can.send(0x01, bytes([byte]))
    assert peer.receive(0x01, REPLY_S, match=lambda f: f.data == b"\x03").data == b"\x03"
    assert [peer.receive(0x01, SILENT_S).data for _ in range(2)] == [b"\x01", b"\x02"]


def test_discard_and_drain_drop_frames(can, peer):
    for can_id in (1, 2, 3):
        can.send(can_id, b"\x00")
    assert peer.receive(3, REPLY_S) is not None   # 1 and 2 are kept on the way
    assert peer.discard(lambda f: f.can_id == 1) == 1
    can.send(4, b"\x00")
    time.sleep(0.01)
    assert peer.drain() == 2   # the kept 2, and 4 still on the bus
    assert peer.receive(timeout=SILENT_S) is None


def test_the_kept_frames_are_bounded(can, peer):
    for _ in range(PENDING_MAX + 5):
        can.send(2, b"\x00")
    can.send(1, b"\x01")
    assert peer.receive(1, REPLY_S).data == b"\x01"
    assert peer.discard(lambda f: True) == PENDING_MAX


def test_empty_and_eight_byte_frames(can, peer):
    can.send(0x01, b"")
    assert peer.receive(0x01, REPLY_S).data == b""
    can.send(0x01, bytes(range(8)))
    assert peer.receive(0x01, REPLY_S).data == bytes(range(8))


def test_a_ninth_byte_or_an_extended_id_is_refused(can):
    with pytest.raises(ValueError):
        can.send(0x01, bytes(9))
    with pytest.raises(ValueError):
        can.send(0x800, b"")


def test_send_after_close_raises(channel):
    bus = CANInterface(channel, "virtual", bitrate=None)
    bus.close()
    with pytest.raises(pycan.CanOperationError):
        bus.send(0x01, b"\x00")
