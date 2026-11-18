"""robotic_arm.arm on the fake boards: send to every board first, then collect."""

from __future__ import annotations

import time
from dataclasses import replace

import pytest
from can import CanOperationError
from conftest import REPLY_S

from robotic_arm.arm import ESTOP_TIMEOUT_S, Arm
from robotic_arm.config import JOINTS
from robotic_arm.joint import JointLimits, LimitError
from robotic_arm.motor_driver import Cmd


@pytest.fixture
def arm(can, fake):
    return Arm(can)


def test_every_joint_of_the_table_is_mapped(arm):
    assert [j.name for j in arm.mapped] == [j.name for j in JOINTS]


def test_emergency_stop_sends_every_stop_before_waiting(arm, fake):
    fake.boards[1].respond = False   # one silent board ...
    start = time.monotonic()
    confirmed = arm.emergency_stop()
    elapsed = time.monotonic() - start
    assert confirmed == {"base_yaw": False} | {j.name: True for j in JOINTS[1:]}
    assert elapsed < ESTOP_TIMEOUT_S + 0.2   # ... costs one wait in all, not one per board
    stops = [(f.can_id, f.data) for f in fake.received[-6:]]
    assert stops == [(i, bytes([0xF7, (i + 0xF7) & 0xFF])) for i in range(1, 7)]   # back to back, every board


def test_a_failed_send_does_not_keep_the_stop_from_the_others(arm, fake, monkeypatch):
    def refuse(_data: bytes) -> None:
        raise CanOperationError("transmit buffer full")

    monkeypatch.setattr(arm.joints["shoulder_pitch"].motor, "send", refuse)
    confirmed = arm.emergency_stop()
    assert confirmed["shoulder_pitch"] is False
    assert all(confirmed[j.name] for j in JOINTS if j.name != "shoulder_pitch")
    assert fake.sent_to(6)[-1][0] == 0xF7


def test_the_priority_stop_is_v1_0_9s_form(arm, fake):
    arm.emergency_stop(priority=True)
    assert {fake.sent_to(i)[-1][:2] for i in range(1, 7)} == {b"\xf7\x01"}


def test_an_unmapped_joint_is_skipped_and_refused(can, fake):
    joints = [replace(j, can_id=None) if j.name == "wrist_roll" else j for j in JOINTS]
    arm = Arm(can, joints)
    assert "wrist_roll" not in arm.emergency_stop()
    assert not fake.sent_to(6)
    with pytest.raises(ValueError):
        arm.move_joints({"wrist_roll": 0.0}, 10.0, 2)


def test_two_joints_cannot_share_a_can_id(can):
    with pytest.raises(ValueError):
        Arm(can, [replace(j, can_id=1) for j in JOINTS[:2]])


def test_collect_returns_each_reply_with_its_timestamp(arm, fake):
    fake.boards[3].respond = False
    start = time.time()
    sent = arm.send_all(bytes([Cmd.READ_STATUS]))
    replies = arm.collect(Cmd.READ_STATUS, sent, timeout=0.2)
    assert set(replies) == set(sent) - {"elbow_pitch"}
    assert all(r.status == 1 and start <= r.timestamp <= time.time() for r in replies.values())


def test_read_positions_turns_counts_into_joint_angles(arm, fake):
    fake.boards[3].position = 1024 * 10   # the elbow: 10 deg past its home
    samples = arm.read_positions(timeout=REPLY_S)
    assert samples["elbow_pitch"].deg == pytest.approx(-31.0)
    assert samples["elbow_pitch"].counts == 10240
    assert samples["base_yaw"].deg == pytest.approx(-169.5)


def test_move_joints_checks_every_target_before_sending_any(arm, fake):
    arm.request_all(bytes([Cmd.ENABLE, 1]))
    sent_before = len(fake.received)
    with pytest.raises(LimitError):
        arm.move_joints({"base_yaw": 0.0, "elbow_pitch": 150.0}, 10.0, 2)   # the elbow's limit is 144
    with pytest.raises(LimitError):
        arm.move_joints({"base_yaw": 0.0}, 10.0, 2, limits={"base_yaw": JointLimits(max_deg=-100.0)})
    time.sleep(0.05)
    assert len(fake.received) == sent_before   # nothing went out


def test_move_joints_moves_each_board_to_its_counts(arm, fake):
    arm.request_all(bytes([Cmd.ENABLE, 1]))
    accepted = arm.move_joints({"elbow_pitch": -40.0, "wrist_pitch": -134.0}, {"elbow_pitch": 16.0, "wrist_pitch": 30.0}, 2)
    assert accepted == {"elbow_pitch": True, "wrist_pitch": True}
    assert fake.boards[3].position == 1024
    assert fake.boards[5].position == round(10 * 16384 / 360 * 4.5)
    assert fake.sent_to(3)[-1][1:3] == (60).to_bytes(2, "big")   # 16 deg/s at 22.5:1 = 60 RPM
