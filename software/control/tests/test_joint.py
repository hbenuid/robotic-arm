"""robotic_arm.joint: degrees <-> counts, and the limit checks - pure arithmetic, no bus."""

from __future__ import annotations

import math
from dataclasses import replace

import pytest

from robotic_arm.config import ENCODER_COUNTS_PER_REV, JOINTS
from robotic_arm.joint import Joint, JointLimits, LimitError

ELBOW = next(j for j in JOINTS if j.name == "elbow_pitch")   # 22.5:1, limits -47..144, home -41


def test_a_motor_turn_is_gear_ratio_of_a_joint_turn():
    for config in JOINTS:
        joint = Joint(config)
        one_motor_turn = joint.zero_deg + 360.0 * config.gear_ratio
        assert joint.deg_to_counts(one_motor_turn) == ENCODER_COUNTS_PER_REV
        assert joint.counts_to_deg(ENCODER_COUNTS_PER_REV) == pytest.approx(one_motor_turn)


def test_the_elbows_numbers():
    joint = Joint(ELBOW)
    assert joint.counts_per_deg == pytest.approx(1024.0)   # 16384 / 360 * 22.5
    assert joint.deg_to_counts(-41.0) == 0                 # the home angle is the encoder's zero after homing
    assert joint.deg_to_counts(-40.0) == 1024
    assert joint.rpm_to_dps(60) == pytest.approx(16.0)     # a motor turn a second is 16 deg/s at 22.5:1
    assert joint.dps_to_rpm(16.0) == pytest.approx(60.0)


def test_sign_and_zero_shift_the_conversion():
    joint = Joint(replace(ELBOW, sign=-1), zero_deg=10.0)
    assert joint.deg_to_counts(11.0) == -1024
    assert joint.counts_to_deg(-1024) == pytest.approx(11.0)


def test_the_callers_limits_narrow_the_arms():
    joint = Joint(ELBOW)
    assert joint.limits() == JointLimits(-47.0, 144.0)
    narrowed = joint.limits(JointLimits(min_deg=-100.0, max_deg=90.0, max_speed_dps=20.0, max_acc=10))
    assert narrowed == JointLimits(-47.0, 90.0, 20.0, 10)


def test_check_move_returns_the_boards_counts_and_rpm():
    assert Joint(ELBOW).check_move(-40.0, 16.0, 2) == (1024, 60)


@pytest.mark.parametrize(("deg", "speed", "acc", "caller"), [
    (145.0, 10.0, 2, None),                          # past the arm's upper limit
    (91.0, 10.0, 2, JointLimits(max_deg=90.0)),      # past the caller's
    (math.nan, 10.0, 2, None),
    (0.0, 30.0, 2, JointLimits(max_speed_dps=20.0)),
    (0.0, 10.0, 11, JointLimits(max_acc=10)),
    (0.0, 10.0, 0, JointLimits(max_acc=10)),         # acc 0 is no ramp: steeper than any cap
    (0.0, 1000.0, 2, None),                          # 3750 RPM: past the board's 3000
])
def test_check_move_refuses_a_broken_limit(deg, speed, acc, caller):
    with pytest.raises(LimitError):
        Joint(ELBOW).check_move(deg, speed, acc, caller)


@pytest.mark.parametrize(("speed", "acc"), [(0.0, 2), (-5.0, 2), (10.0, 256), (10.0, -1)])
def test_check_move_refuses_what_is_no_move(speed, acc):
    with pytest.raises(ValueError):
        Joint(ELBOW).check_move(0.0, speed, acc)


def test_acc_0_passes_when_no_cap_is_set():
    assert Joint(ELBOW).check_move(0.0, 10.0, 0)[0] == 41 * 1024
