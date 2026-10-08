"""The joint limits keep the arm off itself (slow). Over the shoulder's whole range the upper arm itself - j1_link, its
elbow motor + board, the cycloidal drive's turning shell - stays off the j1_coupler fork that holds the drive and off
the base; along the shoulder's upper limit - the arm pitched down in front - the parts beyond the elbow (the forearm
roll drive's motor + board first) stay off the base and the shoulder over the elbow's whole range; at the wrist_pitch
limits the wrist body and the gripper stay off the forearm at any roll; at the elbow's limits the forearm, the wrist and
the gripper stay off the upper arm at any roll and wrist pitch; at any yaw the arm at the shoulder's limits stays off the
base (its motor mount stands out on one side); the forearm roll's limit stops short of its printed hard stop.
Exact distances (BRepExtrema) at poses on the limits, the part pairs whose boxes are apart pruned; a pose with the
arm down in the table the base stands on (below its bottom face) is skipped."""
from __future__ import annotations

import functools
import math

import pytest
from build123d import Axis, Location

from assemblies._occurrences import placement_at, world_rows
from lib import params as PARAMS
from lib.cycloidal import DEFAULT_CONFIG as DRIVE
from lib.datum import BASE_BOTTOM_Y
from lib.forearm import DEFAULT as FOREARM
from lib.yaw_coupler.params import RIM_CLEAR
from robot import frames as F
from tests import built
from tests.helpers import interference

pytestmark = pytest.mark.slow
BEYOND_ELBOW = ("elbow_link", "forearm_link", "wrist_pitch_link", "wrist_roll_link", "jaw_a_link", "jaw_b_link")
_PARENT = {j.child: j for j in F.JOINTS}
_JITTER = ((1.0, 0.7, 1.3), (-1.1, 0.9, -0.6), (0.4, -1.2, 0.8))   # x 0.01 mm: _distance()


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


def _at(links, q: dict) -> list:
    """(label, shape in W) of every part of `links` with the joints at q ({joint: degrees}): each part turned by the
    joints between its link and the base only."""
    out = []
    for link in links:
        chain, at = [], link
        while at in _PARENT:
            chain.append(_PARENT[at].name)
            at = _PARENT[at].parent
        out += _posed(_link_parts(link), *((name, q[name]) for name in chain if q.get(name)))
    return out


def _box_gap(a, b) -> float:
    d = (max(a.min.X - b.max.X, b.min.X - a.max.X, 0.0), max(a.min.Y - b.max.Y, b.min.Y - a.max.Y, 0.0),
         max(a.min.Z - b.max.Z, b.min.Z - a.max.Z, 0.0))
    return math.sqrt(sum(v * v for v in d))


def _boxed(parts) -> list:
    """(label, shape, its bounding box) of every part - a pose measured against many boxed once."""
    return [p if len(p) == 3 else (*p, p[1].bounding_box()) for p in parts]


def _distance(a, b) -> float:
    """a.distance_to(b), its false zeros caught: BRepExtrema now and then reads 0 at one exact pose of two solids that
    neither touch nor overlap (its inside-the-solid check misfires - forearm_roll -140 / -145 and elbow_pitch +92, 1.0 /
    1.2 mm either side). A zero is measured again with the pose moved 0.01 mm three ways and the median kept: a false
    zero belongs to one pose, a real touch stays within the jitter."""
    d = a.distance_to(b)
    if d >= 0.05:
        return d
    return sorted(a.moved(Location(tuple(0.01 * v for v in j))).distance_to(b) for j in _JITTER)[1]


def _closest(movers, fixed, prune: float = 5.0) -> tuple:
    """(distance, mover, fixed part) of the nearest pair; pairs whose boxes are `prune` apart count as that far."""
    best = (math.inf, "", "")
    boxes = _boxed(fixed)
    for ml, ms, mbb in _boxed(movers):
        for fl, fs, fbb in boxes:
            d = _box_gap(mbb, fbb)
            if d < prune:
                d = _distance(ms, fs)
            if d < best[0]:
                best = (d, ml, fl)
    return best


def _in_table(parts) -> bool:
    """A pose with the arm down in the table the base stands on (below its bottom face)."""
    return min(s.bounding_box(optimal=True).min.Y for _, s in parts) < BASE_BOTTOM_Y


def test_the_shoulder_range_keeps_the_upper_arm_off_the_fork_and_the_base():
    """The drive's turning shell and its held parts are coaxial (its own tests): the shoulder link's parts here are the
    fork, its cap and what rides on them outside the drive; every 15 degrees and both limits. The nearest are the fork's
    legs, their running gap (ShellParams.end_plate_gap) off the shell's ends at every angle."""
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
    lo, hi = PARAMS.ELBOW_PITCH_LIMITS_DEG
    checked = []
    for shoulder in (upper, upper - 5.0):
        for elbow in sorted({*range(int(lo), int(hi) + 1, 10), hi}):
            posed = _posed(movers, ("elbow_pitch", elbow), ("shoulder_pitch", shoulder))
            if _in_table(posed):
                continue
            d, ml, fl = _closest(posed, fixed)
            assert d >= 1.0, f"shoulder {shoulder}, elbow {elbow}: {ml} {d:.2f} mm from {fl}"
            checked.append((shoulder, elbow))
    # the folded-back side, where the roll drive's motor + board come nearest, is above the table at the limit
    assert (upper, lo) in checked, checked


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


def test_the_elbow_limits_keep_the_forearm_and_the_gripper_off_the_upper_arm():
    """The forearm, the wrist and the gripper, at any forearm roll and wrist pitch, stay 1 mm off the upper arm, the
    shoulder and the base at the elbow's limits (the roll drive's frame, which turns with the elbow but not the roll:
    tests/forearm/test_roll_drive.py). Those are turned the other way about the elbow axis instead, so one posed forearm
    per roll and wrist pitch serves both limits."""
    beyond = [link for link in BEYOND_ELBOW if link != "elbow_link"]
    behind = _link_parts("upper_arm_link") + _link_parts("shoulder_link") + _link_parts("base_link")
    folded = {elbow: _boxed(_posed(behind, ("elbow_pitch", -elbow))) for elbow in PARAMS.ELBOW_PITCH_LIMITS_DEG}
    roll = PARAMS.FOREARM_ROLL_LIMIT_DEG
    for r in (-roll, -90.0, 0.0, 90.0, roll):
        for w in PARAMS.WRIST_PITCH_LIMITS_DEG:
            movers = _boxed(_at(beyond, {"forearm_roll": r, "wrist_pitch": w}))
            for elbow, fixed in folded.items():
                d, ml, fl = _closest(movers, fixed)
                assert d >= 1.0, f"elbow {elbow}, roll {r}, wrist {w}: {ml} {d:.2f} mm from {fl}"


def test_any_yaw_keeps_the_arm_off_the_base():
    """The base is not round about the base_yaw axis - its motor mount stands out on one side - so the shoulder limits'
    poses (over the elbow's range, those down in the table skipped) are swept round it: every 30 degrees of yaw and the
    limits, everything base_yaw turns stays 1 mm off the base. The base is turned the other way instead, so one posed arm
    serves every yaw. What turns on the axis - j1_coupler and its motor leg, their rim RIM_CLEAR over the base's top
    face, and inside the base the coupler's stub in its bearings on the thrust bearing, the 120T under it
    (tests/test_mounts.py) - keeps those gaps to the base itself and 1 mm to its off-axis parts."""
    riding = ("j1_coupler#1:", "j1_motor_leg#1:")
    inside = ("washer_as6590#2:", "gt2_pulley_120t#1:", "yaw_pulley_screws#1:", "yaw_pulley_nuts#1:")
    in_bore = ("base#1:", "bearing_6806#1:", "bearing_6806#2:", "washer_as6590#1:", "bearing_axk6590#1:")
    shoulder = _link_parts("shoulder_link")
    rims = _boxed([p for p in shoulder if p[0].startswith(riding)])
    turning = rims + _boxed([p for p in shoulder if p[0].startswith(inside)])
    held = [p for p in shoulder if not p[0].startswith(riding + inside)]
    elo, ehi = PARAMS.ELBOW_PITCH_LIMITS_DEG
    poses = [_boxed(held)]
    for s in PARAMS.SHOULDER_PITCH_LIMITS_DEG:
        above = [arm for e in sorted({*range(int(elo), int(ehi) + 1, 15), ehi})
                 if not _in_table(arm := _at(["upper_arm_link", *BEYOND_ELBOW], {"shoulder_pitch": s, "elbow_pitch": e}))]
        assert above, f"shoulder {s}: every elbow pose is down in the table"
        poses += [_boxed(arm) for arm in above]
    lim = PARAMS.BASE_YAW_LIMIT_DEG
    for yaw in sorted({*range(-(int(lim) // 30) * 30, int(lim) + 1, 30), -lim, lim}):
        base = _boxed(_posed(_link_parts("base_link"), ("base_yaw", -yaw)))
        d, ml, fl = _closest(turning, [p for p in base if not p[0].startswith(in_bore)])
        assert d >= 1.0, f"yaw {yaw}: {ml} {d:.2f} mm from {fl}"
        d, ml, fl = _closest(rims, [p for p in base if p[0].startswith("base#1:")])
        assert d >= RIM_CLEAR - 0.01, f"yaw {yaw}: {ml} {d:.2f} mm over the base's top face"
        for movers in poses:
            d, ml, fl = _closest(movers, base)
            assert d >= 1.0, f"yaw {yaw}: {ml} {d:.2f} mm from {fl}"


def test_the_forearm_roll_limit_stops_short_of_the_hard_stop():
    """The roll's hard stop - the lug on the shaft's flange against the post on the frame's tower - lies between the limit
    and 180 - stop_deg_width: at +/- FOREARM_ROLL_LIMIT_DEG the shaft stands clear of the frame, at the nominal angle the
    lug is into the post (their parallel sides meet before it: lib/forearm/params.py)."""
    shaft = [p for p in _link_parts("forearm_link") if p[0].endswith(":forearm_roll_shaft")]
    block = next(s for label, s in _link_parts("elbow_link") if label.endswith(":forearm_roll_block"))
    limit, nominal = PARAMS.FOREARM_ROLL_LIMIT_DEG, 180.0 - FOREARM.drive.stop_deg_width
    for sign in (1.0, -1.0):
        (_, at_limit), = _posed(shaft, ("forearm_roll", sign * limit))
        gap = _distance(at_limit, block)
        assert gap >= 0.5, f"roll {sign * limit:+.0f}: the shaft {gap:.2f} mm from the frame"
        (_, at_nominal), = _posed(shaft, ("forearm_roll", sign * nominal))
        vol = interference(at_nominal, block)
        assert vol > 1.0, f"roll {sign * nominal:+.0f}: the lug only {vol:.2f} mm^3 into the post - no stop"
