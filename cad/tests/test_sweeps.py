"""The joint limits keep the arm off itself (slow). Over the shoulder's whole range the upper arm itself - j1_link, its
elbow motor + board, the cycloidal drive's turning shell - stays off the j1_coupler fork that holds the drive and off
the base; along the shoulder's upper limit - the arm pitched down in front - the parts beyond the elbow (the forearm
roll drive's motor + board first) stay off the base and the shoulder over the elbow's whole range; at the wrist_pitch
limits the wrist body and the gripper stay off the forearm at any roll; at the elbow's limits the forearm, the wrist and
the gripper stay off the upper arm at any roll and wrist pitch; at any yaw the arm at the shoulder's limits stays off the
base (its motor mount stands out on one side); the forearm and the wrist roll's limits stop short of their printed hard
stops; the gripper rolls clear of the wrist roll drive's frame and the forearm; every MKS board's plugs keep their room
in front of their headers over the joints' ranges (an overlap, not a distance: `_plug_overlap`).
Exact distances (BRepExtrema) at poses on the limits, for every part pair that could come nearer than the limit: a pair
whose boxes are the limit apart is not measured - the world boxes, each part's own box turned with it (`_oriented`) and,
for a part turning about one joint through many angles, its box in that joint's frame (`_hulls`); a pose with the arm
down in the table the base stands on (below its bottom face) is skipped."""
from __future__ import annotations

import functools
import math
from typing import NamedTuple

import pytest
from build123d import Axis, Compound, Location, Pos, Solid, Vector
from OCP.gp import gp_Trsf
from OCP.TopLoc import TopLoc_Location

from assemblies._occurrences import placement_at, world_rows
from lib import params as PARAMS
from lib.cycloidal import DEFAULT_CONFIG as DRIVE
from lib.datum import BASE_BOTTOM_Y
from lib.forearm import DEFAULT as FOREARM
from lib.wrist import DEFAULT as WRIST
from lib.yaw_coupler.params import RIM_CLEAR
from robot import frames as F
from tests import built
from tests.helpers import interference

pytestmark = pytest.mark.slow
CLEAR = 1.0   # mm: what a part keeps off another link's parts
BEYOND_ELBOW = ("elbow_link", "forearm_link", "wrist_pitch_link", "wrist_roll_link", "jaw_a_link", "jaw_b_link")
_PARENT = {j.child: j for j in F.JOINTS}
_JITTER = ((1.0, 0.7, 1.3), (-1.1, 0.9, -0.6), (0.4, -1.2, 0.8))   # x 0.01 mm: _distance()


@functools.cache
def _link_parts(link: str) -> tuple:
    """(label, shape in W) of every part of a robot link, at the capture pose."""
    return tuple((f"{key}:{part}", built.part(part).moved(placement_at(part, world)))
                 for key in F.LINKS[link] for part, _role, world in world_rows(key))


def _turn(joint: str, deg: float) -> Location:
    """`deg` degrees about `joint`'s axis, in W - what Shape.rotate() moves a shape by."""
    j = F.JOINT_BY_NAME[joint]
    t = gp_Trsf()
    t.SetRotation(Axis(j.origin_w, j.axis_w).wrapped, math.radians(deg))
    return Location(TopLoc_Location(t))


def _moved(shape, loc: Location):
    """`shape` moved by `loc`: Shape.moved() / .rotate() without the copy of the whole geometry they make first and throw
    away (Shape.__deepcopy__) - a plain Solid / Compound round the moved TopoDS, with no label, colour or tree (the sweeps
    label their parts themselves)."""
    return Compound.cast(shape.wrapped.Moved(loc.wrapped))


def _posed(parts, *turns) -> list:
    """`parts` turned about joints, `turns` = ((joint, degrees), ...) in order, the distal joint first: one move each."""
    if not turns:
        return list(parts)
    loc = Location()
    for name, deg in turns:
        loc = _turn(name, deg) * loc
    return [(label, _moved(shape, loc)) for label, shape in parts]


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


def _hulls(parts, joint: str) -> list:
    """(label, box) of every part: its bounding box in `joint`'s frame, as a solid. It holds the part and turns with it
    about the joint's axis, so its distance to a fixed part bounds the part's own from below - snug at every angle,
    where the world box of a part turned off the world's axes grows."""
    frame = F.joint_frame_world(joint)
    out = []
    for label, shape in parts:
        bb = shape.moved(frame.inverse()).bounding_box()
        out.append((label, Solid.make_box(bb.size.X, bb.size.Y, bb.size.Z).moved(frame * Pos(bb.min.X, bb.min.Y, bb.min.Z))))
    return out


@functools.cache
def _own_box(part: str) -> tuple:
    """(low, high) corners of a part's bounding box in its own frame - its shape as built, before any location."""
    bb = Compound.cast(built.part(part).wrapped.Located(TopLoc_Location())).bounding_box()
    return (bb.min.X, bb.min.Y, bb.min.Z), (bb.max.X, bb.max.Y, bb.max.Z)


def _oriented(label: str, shape) -> tuple:
    """(centre, axes, half sizes) in W of the part's own box (`_own_box`), carried by its shape's location: it moves with
    the part, so it stays as snug at any pose, where the world box of a part turned off the world's axes grows."""
    lo, hi = _own_box(label.rsplit(":", 1)[-1])
    t = shape.wrapped.Location().Transformation()
    r = [[t.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)]
    c = [(lo[k] + hi[k]) / 2 for k in range(3)]
    centre = [sum(r[i][k] * c[k] for k in range(3)) + t.Value(i + 1, 4) for i in range(3)]
    return centre, [[r[i][k] for i in range(3)] for k in range(3)], [(hi[k] - lo[k]) / 2 for k in range(3)]


def _apart(a, b, margin: float) -> bool:
    """The oriented boxes a and b (`_oriented`) are more than `margin` apart: a separating axis - one of the 15 of two
    boxes - with that gap between their shadows on it. A box `margin` bigger all round holds every point within `margin`
    of a box, so the parts inside are more than `margin` apart too. The axis of two near-parallel edges is skipped (too
    short to trust; skipping one only ever measures more)."""
    (ca, xa, ha), (cb, xb, hb) = a, b
    d = [cb[i] - ca[i] for i in range(3)]
    for ax in [*xa, *xb, *(_cross(u, v) for u in xa for v in xb)]:
        n = math.sqrt(_dot(ax, ax))
        if n < 1e-6:
            continue
        span = sum(ha[k] * abs(_dot(xa[k], ax)) + hb[k] * abs(_dot(xb[k], ax)) for k in range(3))
        if abs(_dot(d, ax)) > span + margin * n:
            return True
    return False


def _dot(u, v) -> float:
    return u[0] * v[0] + u[1] * v[1] + u[2] * v[2]


def _cross(u, v) -> list:
    return [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]


def _closest(movers, fixed, limit: float, hulls=None) -> tuple:
    """(distance, mover, fixed part) of the nearest pair, exact wherever it is under `limit`. A pair whose world boxes
    are `limit` apart counts as their gap; one whose own boxes (`_oriented`) are, as `limit`; one whose mover's hull
    (`_hulls`, posed as the movers, one each) is, as the hull's distance - each bounds the pair's distance from below, so
    such a pair cannot break the limit. The exact distance only under all three."""
    best = (math.inf, "", "")
    boxes = [(*p, _oriented(p[0], p[1])) for p in _boxed(fixed)]
    for i, (ml, ms, mbb) in enumerate(_boxed(movers)):
        mob = _oriented(ml, ms)
        for fl, fs, fbb, fob in boxes:
            d = _box_gap(mbb, fbb)
            if d < limit and _apart(mob, fob, limit):
                d = limit
            if d < limit and hulls is not None:
                d = hulls[i][1].distance_to(fs)
            if d < limit:
                d = _distance(ms, fs)
            if d < best[0]:
                best = (d, ml, fl)
    return best


def _in_table(parts) -> bool:
    """A pose with the arm down in the table the base stands on (below its bottom face); `parts` boxed (`_boxed`)."""
    return min(bb.min.Y for _, _, bb in parts) < BASE_BOTTOM_Y


def test_the_shoulder_range_keeps_the_upper_arm_off_the_fork_and_the_base():
    """The drive's turning shell and its held parts are coaxial (its own tests): the shoulder link's parts here are the
    fork, its motor leg and what rides on them outside the drive; every 15 degrees and both limits. The nearest are the
    fork's legs, their running gap (ShellParams.end_plate_gap) off the shell's ends at every angle."""
    fixed = [*_link_parts("base_link"), *(p for p in _link_parts("shoulder_link") if not p[0].startswith("cycloidal_drive"))]
    movers = _link_parts("upper_arm_link")
    gap = DRIVE.shell.end_plate_gap - 0.01
    lo, hi = PARAMS.SHOULDER_PITCH_LIMITS_DEG
    for shoulder in sorted({lo, hi, *range(int(lo), int(hi) + 1, 15)}):
        d, ml, fl = _closest(_posed(movers, ("shoulder_pitch", shoulder)), fixed, gap)
        assert d >= gap, f"shoulder {shoulder}: {ml} {d:.2f} mm from {fl}"


def test_the_shoulder_limit_keeps_the_forearm_off_the_base_and_the_shoulder():
    fixed = _link_parts("base_link") + _link_parts("shoulder_link")
    movers = [p for link in BEYOND_ELBOW for p in _link_parts(link)]
    upper = PARAMS.SHOULDER_PITCH_LIMITS_DEG[1]
    lo, hi = PARAMS.ELBOW_PITCH_LIMITS_DEG
    checked = []
    for shoulder in (upper, upper - 5.0):
        for elbow in sorted({*range(int(lo), int(hi) + 1, 10), hi}):
            posed = _boxed(_posed(movers, ("elbow_pitch", elbow), ("shoulder_pitch", shoulder)))
            if _in_table(posed):
                continue
            d, ml, fl = _closest(posed, fixed, CLEAR)
            assert d >= CLEAR, f"shoulder {shoulder}, elbow {elbow}: {ml} {d:.2f} mm from {fl}"
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
            d, ml, fl = _closest(movers, fixed, CLEAR)
            assert d >= CLEAR, f"wrist {wrist}, roll {roll}: {ml} {d:.2f} mm from {fl}"


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
                d, ml, fl = _closest(movers, fixed, CLEAR)
                assert d >= CLEAR, f"elbow {elbow}, roll {r}, wrist {w}: {ml} {d:.2f} mm from {fl}"


def test_any_yaw_keeps_the_arm_off_the_base():
    """The base is not round about the base_yaw axis - its motor mount stands out on one side - so the shoulder limits'
    poses (over the elbow's range, those down in the table skipped) are swept round it: every 30 degrees of yaw and the
    limits, everything base_yaw turns stays 1 mm off the base. The base is turned the other way instead, so one posed arm
    serves every yaw. What turns on the axis - j1_coupler and its motor leg, their rim RIM_CLEAR over the base's top
    face, and inside the base the coupler's stub in its bearings on the thrust bearing, the 120T under it
    (tests/test_mounts.py) - keeps those gaps to the base itself and 1 mm to its off-axis parts."""
    riding = ("j1_coupler#1:", "j1_motor_leg#1:")
    inside = ("washer_as6590#2:", "gt2_pulley_120t#1:", "yaw_pulley_screws#1:", "yaw_pulley_nuts#1:")
    in_bore = ("base#1:", "bearing_6806#1:", "bearing_6806#2:", "washer_as6590#1:", "bearing_axk6590#1:",
               "yaw_belt#1:")   # the belt rides the 120T (tests/test_belts.py)
    shoulder = _link_parts("shoulder_link")
    rims = _boxed([p for p in shoulder if p[0].startswith(riding)])
    turning = rims + _boxed([p for p in shoulder if p[0].startswith(inside)])
    held = [p for p in shoulder if not p[0].startswith(riding + inside)]
    elo, ehi = PARAMS.ELBOW_PITCH_LIMITS_DEG
    poses = [_boxed(held)]
    for s in PARAMS.SHOULDER_PITCH_LIMITS_DEG:
        arms = (_boxed(_at(["upper_arm_link", *BEYOND_ELBOW], {"shoulder_pitch": s, "elbow_pitch": e}))
                for e in sorted({*range(int(elo), int(ehi) + 1, 15), ehi}))
        above = [arm for arm in arms if not _in_table(arm)]
        assert above, f"shoulder {s}: every elbow pose is down in the table"
        poses += above
    rim = RIM_CLEAR - 0.01
    lim = PARAMS.BASE_YAW_LIMIT_DEG
    for yaw in sorted({*range(-(int(lim) // 30) * 30, int(lim) + 1, 30), -lim, lim}):
        base = _boxed(_posed(_link_parts("base_link"), ("base_yaw", -yaw)))
        d, ml, fl = _closest(turning, [p for p in base if not p[0].startswith(in_bore)], CLEAR)
        assert d >= CLEAR, f"yaw {yaw}: {ml} {d:.2f} mm from {fl}"
        d, ml, fl = _closest(rims, [p for p in base if p[0].startswith("base#1:")], rim)
        assert d >= rim, f"yaw {yaw}: {ml} {d:.2f} mm over the base's top face"
        for movers in poses:
            d, ml, fl = _closest(movers, base, CLEAR)
            assert d >= CLEAR, f"yaw {yaw}: {ml} {d:.2f} mm from {fl}"


def test_the_wrist_roll_limit_stops_short_of_the_hard_stop():
    """The wrist roll's hard stop - the lug on its shaft's flange against the post in wrist_link's bay - lies between the
    limit and 180 - stop_deg_width, as the forearm roll's."""
    shaft = [p for p in _link_parts("wrist_roll_link") if p[0].endswith(":wrist_roll_shaft")]
    frame = next(s for label, s in _link_parts("wrist_pitch_link") if label.endswith(":wrist_link"))
    limit, nominal = PARAMS.WRIST_ROLL_LIMIT_DEG, 180.0 - WRIST.roll.stop_deg_width
    for sign in (1.0, -1.0):
        (_, at_limit), = _posed(shaft, ("wrist_roll", sign * limit))
        gap = _distance(at_limit, frame)
        assert gap >= 0.5, f"roll {sign * limit:+.0f}: the shaft {gap:.2f} mm from the frame"
        (_, at_nominal), = _posed(shaft, ("wrist_roll", sign * nominal))
        vol = interference(at_nominal, frame)
        assert vol > 1.0, f"roll {sign * nominal:+.0f}: the lug only {vol:.2f} mm^3 into the post - no stop"


def test_the_gripper_rolls_clear_of_the_wrist():
    """The gripper rolled through its range - every 15 degrees and the limits - stays 1 mm off the wrist body, the wrist
    roll drive's stator and the forearm, the wrist level (the drive's rotor, which turns in its bearings and drives the
    belt, keeps its run_gap: tests/wrist/test_roll_drive.py)."""
    rolling = [p for link in ("wrist_roll_link", "jaw_a_link", "jaw_b_link") for p in _link_parts(link)
               if not p[0].endswith((":wrist_roll_pulley", ":wrist_roll_shaft"))]
    hulls = _hulls(rolling, "wrist_roll")
    fixed = _boxed(list(_link_parts("wrist_pitch_link")) + list(_link_parts("forearm_link")))
    lim = PARAMS.WRIST_ROLL_LIMIT_DEG
    for roll in sorted({*range(-(int(lim) // 15) * 15, int(lim) + 1, 15), -lim, lim}):
        turn = ("wrist_roll", roll)
        d, ml, fl = _closest(_posed(rolling, turn), fixed, CLEAR, hulls=_posed(hulls, turn))
        assert d >= CLEAR, f"roll {roll}: {ml} {d:.2f} mm from {fl}"


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


def _steps(lo: float, hi: float, step: int) -> list[float]:
    """Every `step` degrees between the limits, and the limits."""
    return sorted({lo, hi, *range(math.ceil(lo / step) * step, math.floor(hi) + 1, step)})


def _grid(**axes) -> list[dict]:
    """Every combination of the joints' angles: _grid(elbow_pitch=[...], forearm_roll=[...])."""
    out = [{}]
    for name, angles in axes.items():
        out = [{**q, name: a} for q in out for a in angles]
    return out


def _chain(link: str, q: dict) -> tuple:
    """The (joint, degrees) turns between `link` and the base, the distal joint first (as _at() turns a link)."""
    turns, at = [], link
    while at in _PARENT:
        if q.get(_PARENT[at].name):
            turns.append((_PARENT[at].name, q[_PARENT[at].name]))
        at = _PARENT[at].parent
    return tuple(turns)


def _plug_rooms(link: str) -> list:
    """(label, room in W) of every MKS board's three plugs on a robot link: each header's outline carried MKS_PLUG_ROOM
    straight out from its face (lib/motors.py), on the board as its mount turned it."""
    rooms = []
    for key in F.LINKS[link]:
        for part, _role, world in world_rows(key):
            if part != "mks_servo42d":
                continue
            for name, ((x0, y0, z0, x1, y1, z1), (ux, uy)) in PARAMS.MKS_SERVO42D_HEADERS.items():
                d = PARAMS.MKS_PLUG_ROOM[name]
                xa, xb = ((x1, x1 + d) if ux > 0 else (x0 - d, x0)) if ux else (x0, x1)
                ya, yb = ((y1, y1 + d) if uy > 0 else (y0 - d, y0)) if uy else (y0, y1)
                room = Solid.make_box(xb - xa, yb - ya, z1 - z0).moved(Location((xa, ya, z0)))
                rooms.append((f"{key}:{part} {name}", room.moved(placement_at(part, world))))
    return rooms


_EPS = 1e-3   # mm: a holder (`_holder`) this far clear of a room or the table decides for its part's own bounding box


class _Corners(NamedTuple):
    """A box's low and high corners - what _box_gap() reads of a bounding box."""
    min: Vector
    max: Vector


def _holder(label: str, shape) -> _Corners:
    """The world box round the part's own box (`_oriented`): a box holding the part, carried with it, so it holds the
    part's bounding_box() at any pose - and costs no kernel call, where the optimal box of a vendor motor's splines
    costs a quarter of a second."""
    c, axes, h = _oriented(label, shape)
    e = [sum(abs(axes[k][i]) * h[k] for k in range(3)) for i in range(3)]
    return _Corners(Vector(*(c[i] - e[i] for i in range(3))), Vector(*(c[i] + e[i] for i in range(3))))


def _plug_overlap(room, part) -> float:
    """interference(room, part); a hit measured again with the room moved 0.01 mm two ways and the median kept (as
    _distance(): a boolean that misfires at one exact pose)."""
    v = interference(room, part)
    if v < 0.01:
        return v
    return sorted([v, *(interference(_moved(room, Location(tuple(0.01 * c for c in j))), part) for j in _JITTER[:2])])[1]


_SH, _EL, _WR = PARAMS.SHOULDER_PITCH_LIMITS_DEG, PARAMS.ELBOW_PITCH_LIMITS_DEG, PARAMS.WRIST_PITCH_LIMITS_DEG
_FR, _YAW = PARAMS.FOREARM_ROLL_LIMIT_DEG, PARAMS.BASE_YAW_LIMIT_DEG
# Per board link: the links that move relative to it, with the poses of the joints between - group by group.
_PLUG_SWEEPS = {
    # the base's rooms stay inside the motor mount and the base, which the arm keeps off (the yaw test above)
    "base_link": [(("base_link",), [{}])],
    "shoulder_link": [
        (("shoulder_link",), [{}]),
        (("base_link",), _grid(base_yaw=_steps(-_YAW, _YAW, 15))),
        (("upper_arm_link",), _grid(shoulder_pitch=_steps(*_SH, 10))),
        (BEYOND_ELBOW, _grid(shoulder_pitch=_steps(*_SH, 30), elbow_pitch=_steps(*_EL, 30), forearm_roll=[-_FR, 0.0, _FR]))],
    "upper_arm_link": [
        (("upper_arm_link",), [{}]),
        (("shoulder_link", "base_link"), _grid(base_yaw=[0.0, 90.0, 180.0], shoulder_pitch=_steps(*_SH, 15))),
        (("elbow_link",), _grid(elbow_pitch=_steps(*_EL, 5))),
        (BEYOND_ELBOW[1:], _grid(elbow_pitch=_steps(*_EL, 15), forearm_roll=_steps(-_FR, _FR, 45),
                                 wrist_pitch=[_WR[0], 0.0, _WR[1]]))],
    "elbow_link": [
        (("elbow_link",), [{}]),
        (("upper_arm_link", "shoulder_link"), _grid(elbow_pitch=_steps(*_EL, 10))),
        (BEYOND_ELBOW[1:], _grid(forearm_roll=_steps(-_FR, _FR, 30), wrist_pitch=_steps(*_WR, 50)))],
    "forearm_link": [
        (("forearm_link",), [{}]),
        (BEYOND_ELBOW[2:], _grid(wrist_pitch=_steps(*_WR, 5), wrist_roll=[0.0, 90.0, 180.0, 270.0])),
        (("elbow_link", "upper_arm_link"), _grid(forearm_roll=_steps(-_FR, _FR, 5), elbow_pitch=[_EL[0], 0.0, _EL[1]])),
        (("shoulder_link", "base_link"), _grid(forearm_roll=_steps(-_FR, _FR, 45), elbow_pitch=_steps(*_EL, 30),
                                               shoulder_pitch=_steps(*_SH, 30)))],
}


@pytest.mark.parametrize("link", list(_PLUG_SWEEPS))
def test_the_mks_boards_keep_their_plug_room(link):
    """Every MKS board's three plugs keep MKS_PLUG_ROOM straight out from their headers (lib/motors.py) at every pose:
    nothing of the board's own link in the rooms, nor of the links that move relative to it over their joints' ranges
    (_PLUG_SWEEPS; poses with a part they turn down in the table skipped). The boards' quarter turns on their motors
    (lib/mounts.py BOARD_TURNS) and the roll frame's plug slots (lib/forearm/layout.py plug_slots) keep them clear.
    Each link is posed once per angles of the joints that turn it, and a part's optimal bounding box is made only where
    its holder (`_holder`) cannot decide - once per pose of its link."""
    rooms = _plug_rooms(link)
    assert rooms, link
    boards = {label.split(" ")[0] for label, _room in rooms}   # the boards themselves (their own covers frame the headers)
    posed, boxes = {}, {}

    def at(name: str, q: dict) -> list:
        """(label, shape, holder) of every part of a link at q (as _at() poses it)."""
        turns = _chain(name, q)
        if (name, turns) not in posed:
            posed[name, turns] = [(label, s, _holder(label, s)) for label, s in _posed(_link_parts(name), *turns)]
        return posed[name, turns]

    def box(label: str, shape, holder: _Corners):
        """shape.bounding_box(), kept (`posed` keeps the shapes, so their ids stay theirs) - inside its holder to half of
        _EPS, so a holder _EPS clear of a room or the table leaves the box clear of it too."""
        if id(shape) not in boxes:
            bb = boxes[id(shape)] = shape.bounding_box()
            lo, hi, blo, bhi = (tuple(v) for v in (holder.min, holder.max, bb.min, bb.max))
            assert all(lo[i] - _EPS / 2 < blo[i] and bhi[i] < hi[i] + _EPS / 2
                       for i in range(3)), f"{label}: its bounding box sticks out of its holder"
        return boxes[id(shape)]

    def down(names, q: dict) -> bool:
        """_in_table() over the parts q turns: those of the links `names` with a joint turned between them and the base.
        What it leaves where the capture put it is above the table - but the base, standing on it, has a box a float-noise
        hair under its bottom face, which read as down in it and skipped every pose of a group that holds the base."""
        return any(box(*p).min.Y < BASE_BOTTOM_Y for name in names if _chain(name, q) for p in at(name, q)
                   if p[2].min.Y < BASE_BOTTOM_Y + _EPS)

    for links, poses in _PLUG_SWEEPS[link]:
        for q in poses:
            others = [p for name in links for p in at(name, q) if p[0] not in boards]
            if q and down({*links, link}, q):
                continue
            for rl, room, rbb in _boxed(_posed(rooms, *_chain(link, q))):
                for pl, part, holder in others:
                    if _box_gap(rbb, holder) > _EPS or _box_gap(rbb, box(pl, part, holder)) > 0.0:
                        continue
                    v = _plug_overlap(room, part)
                    assert v < 0.01, f"{rl}: {pl} {v:.2f} mm^3 in the plug's room at {q}"
