"""The joint limits keep the arm off itself (slow). Over the shoulder's whole range the upper arm itself - j1_link, its
elbow motor + board, the cycloidal drive's turning shell - stays off the j1_coupler fork that holds the drive and off
the base; along the shoulder's upper limit - the arm pitched down in front - the parts beyond the elbow (the forearm
roll drive's motor + board first) stay off the base and the shoulder over the elbow's whole range; at the wrist_pitch
limits the wrist body and the gripper stay off the forearm at any roll.
Exact distances (BRepExtrema) at poses on the limits, the part pairs whose boxes are apart pruned; a pose with the
arm down in the table the base stands on (below its bottom face) is skipped."""
from __future__ import annotations

import functools
import math

import pytest
from build123d import Axis

from assemblies._occurrences import placement_at, world_rows
from lib import params as PARAMS
from lib.cycloidal import DEFAULT_CONFIG as DRIVE
from lib.datum import BASE_BOTTOM_Y
from robot import frames as F
from tests import built

pytestmark = pytest.mark.slow
BEYOND_ELBOW = ("elbow_link", "forearm_link", "wrist_pitch_link", "wrist_roll_link", "jaw_a_link", "jaw_b_link")


@functools.cache
def _link_parts(link: str) -> tuple:
    """(label, shape in W) of every part of a robot link, at the capture pose."""
    return tuple((f"{key}:{part}", built.part(part).moved(placement_at(part, world)))
                 for key in F.LINKS[link] for part, _role, world in world_rows(key))


def _posed(parts, *turns) -> list:
    """`parts` turned about joints, `turns` = ((joint, degrees), ...) in order, the distal joint first."""
    out = []
    for label, shape in parts:
        for name, deg in turns:
            j = F.JOINT_BY_NAME[name]
            shape = shape.rotate(Axis(j.origin_w, j.axis_w), deg)
        out.append((label, shape))
    return out


def _box_gap(a, b) -> float:
    d = (max(a.min.X - b.max.X, b.min.X - a.max.X, 0.0), max(a.min.Y - b.max.Y, b.min.Y - a.max.Y, 0.0),
         max(a.min.Z - b.max.Z, b.min.Z - a.max.Z, 0.0))
    return math.sqrt(sum(v * v for v in d))


def _closest(movers, fixed, prune: float = 5.0) -> tuple:
    """(distance, mover, fixed part) of the nearest pair; pairs whose boxes are `prune` apart count as that far."""
    best = (math.inf, "", "")
    boxes = [(label, shape, shape.bounding_box()) for label, shape in fixed]
    for ml, ms in movers:
        mbb = ms.bounding_box()
        for fl, fs, fbb in boxes:
            d = _box_gap(mbb, fbb)
            if d < prune:
                d = ms.distance_to(fs)
            if d < best[0]:
                best = (d, ml, fl)
    return best


def test_the_shoulder_range_keeps_the_upper_arm_off_the_fork_and_the_base():
    """The drive's turning shell and its held parts are coaxial (its own tests): the shoulder link's parts here are the
    fork, its cap and what rides on them outside the drive; every 15 degrees and both limits. The nearest is the fork's
    end plate, its running gap (ShellParams.end_plate_gap) under j1_link at every angle."""
    fixed = [*_link_parts("base_link"), *(p for p in _link_parts("shoulder_link") if not p[0].startswith("cycloidal_drive"))]
    movers = _link_parts("upper_arm_link")
    lo, hi = PARAMS.SHOULDER_PITCH_LIMITS_DEG
    for shoulder in sorted({lo, hi, *range(int(lo), int(hi) + 1, 15)}):
        d, ml, fl = _closest(_posed(movers, ("shoulder_pitch", shoulder)), fixed)
        assert d >= DRIVE.shell.end_plate_gap - 0.01, f"shoulder {shoulder}: {ml} {d:.2f} mm from {fl}"


def test_the_shoulder_limit_keeps_the_forearm_off_the_base_and_the_shoulder():
    fixed = _link_parts("base_link") + _link_parts("shoulder_link")
    movers = [p for link in BEYOND_ELBOW for p in _link_parts(link)]
    upper = PARAMS.SHOULDER_PITCH_LIMITS_DEG[1]
    checked = []
    for shoulder in (upper, upper - 5.0):
        for elbow in range(-int(PARAMS.ELBOW_PITCH_LIMIT_DEG), int(PARAMS.ELBOW_PITCH_LIMIT_DEG) + 1, 10):
            posed = _posed(movers, ("elbow_pitch", elbow), ("shoulder_pitch", shoulder))
            if min(s.bounding_box(optimal=True).min.Y for _, s in posed) < BASE_BOTTOM_Y:
                continue               # the arm is down in the table
            d, ml, fl = _closest(posed, fixed)
            assert d >= 1.0, f"shoulder {shoulder}, elbow {elbow}: {ml} {d:.2f} mm from {fl}"
            checked.append((shoulder, elbow))
    # the folded-back side, where the roll drive's motor + board come nearest, is above the table at the limit
    assert (upper, -int(PARAMS.ELBOW_PITCH_LIMIT_DEG)) in checked, checked


def test_the_wrist_pitch_limits_keep_the_wrist_off_the_forearm():
    fixed = _link_parts("forearm_link") + _link_parts("elbow_link")
    on_axis = ("gt2_pulley_90t", "wrist_pulley", "j3_coupler")   # they turn in the forearm's bearings, on the axis
    pitching = [p for p in _link_parts("wrist_pitch_link") if not p[0].startswith(on_axis)]
    rolling = [p for link in ("wrist_roll_link", "jaw_a_link", "jaw_b_link") for p in _link_parts(link)]
    for wrist in PARAMS.WRIST_PITCH_LIMITS_DEG:
        for roll in (0, 90, 180, 270):
            movers = _posed(pitching, ("wrist_pitch", wrist)) + _posed(rolling, ("wrist_roll", roll), ("wrist_pitch", wrist))
            d, ml, fl = _closest(movers, fixed)
            assert d >= 1.0, f"wrist {wrist}, roll {roll}: {ml} {d:.2f} mm from {fl}"
