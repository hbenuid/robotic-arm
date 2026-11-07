"""The gripper module (assemblies/gripper.py, lib/gripper/layout.py): where it puts its parts, its totals, its fits - the
racks in mesh with the pinion and clear of the base over the jaws' travel - and the cage on the connector."""
import math

import pytest

from assemblies import gripper
from lib import datum
from lib.gripper import DEFAULT as G
from lib.gripper import layout
from lib.params import JAW_TRAVEL_MM
from tests.helpers import interference, is_inside


def test_layout_numbers():
    assert layout.x0() == pytest.approx(0.75)        # the case between the posts (40.5 apart), its spline 10.25 off its middle
    z = layout.stations()
    assert (z["tab_face"], z["tab_back"], z["servo_back"], z["spline_end"]) == pytest.approx((22.0, 24.4, 53.2, 8.0))
    assert (z["back_in"], z["seat"], z["pinion"], z["rack"], z["jaw"], z["tips"]) == pytest.approx(
        (53.4, 58.4, 4.25, 10.0, -0.3, -45.3))
    # the spline in the pinion's press bore, short of its floor: from spline_z to the top
    assert z["pinion"] + G.pinion.spline_z < z["spline_end"] < z["pinion"] + G.pinion.width
    assert layout.tool0() == pytest.approx((0.0, 0.0, 103.7))   # the fingertips' midpoint on R', 103.7 past the connector's plate
    assert layout.jaw_origin() == pytest.approx((0.0, 0.0, 48.4))


def test_rows_are_the_layouts():
    assert gripper.OCCURRENCES == layout.rows()
    labels = [p if r is None else f"{p}:{r}" for p, r, _ in gripper.OCCURRENCES]
    assert sorted(labels) == sorted({m for b in gripper.BODIES.values() for m in b} - {"mg996r_servo"} | {"mg996r_servo:gripper"})
    for body in gripper.BODIES:
        assert {lab for lab in labels if gripper.in_body(lab, body)}, body


def test_the_rows_turn_as_meant():
    """Each row's rotation as to_location() reads it: the base turned over (the fingers along +Z), the connector's plate's
    hub side toward the pulley (-Z), the servo's spline toward the plate (+Z, on R')."""
    rot = {(p, r): datum.rotation(f[1]) for p, r, f in gripper.OCCURRENCES}

    def col(m, i):
        return tuple(round(m[k][i], 9) for k in range(3))

    assert col(rot[("gripper_base", None)], 2) == (0.0, 0.0, -1.0)           # the posts (base +z) back toward the wrist
    assert col(rot[("gripper_base", None)], 0) == (0.0, 1.0, 0.0)            # the jaws' travel on +Y
    assert col(rot[("gripper_j3_connector", None)], 1) == (0.0, 0.0, -1.0)   # its hub (+Y) toward the pulley
    assert col(rot[("mg996r_servo", "gripper")], 1) == (0.0, 0.0, 1.0)       # its +Y (toward the spline) toward the fingers
    pos = {(p, r): f[0] for p, r, f in gripper.OCCURRENCES}
    for key in (("gripper_pinion", None), ("gripper_rack", "a"), ("gripper_jaw", "b")):
        assert pos[key][:2] == pytest.approx((0.0, 0.0)), key                # on R'


def test_travel_stops_short_of_the_slots_ends():
    """A rack's post's round end is concentric with its slot's end disc, slot_end_r - post half-width bigger: each rack
    stops there - and JAW_TRAVEL_MM short of it either way."""
    b, r, x0 = G.base, G.rack, layout.x0()
    post_r = (r.post_y[1] - r.post_y[0]) / 2.0
    assert sum(r.post_y) / 2.0 == pytest.approx(b.slot_end_y)                # the posts on the slots' middle line
    reach = b.slot_x + (b.slot_end_r - post_r)                                # a post end's centre's furthest |x|
    ends = [e for (x0_, x1_) in r.posts_x for e in (x0_ + post_r, x1_ - post_r)]   # in the rack's frame

    def clear(t):   # closing by t: rack a (turned over, x -> -x) at x0 + t, rack b at x0 - t
        return all(abs(x0 + t - e) <= reach + 1e-9 and abs(x0 - t + e) <= reach + 1e-9 for e in ends)

    steps = [k / 1000.0 for k in range(15000)]
    closing = max(t for t in steps if clear(t))
    opening = max(t for t in steps if clear(-t))
    assert (closing, opening) == pytest.approx((10.86, 10.84), abs=1e-3)
    assert JAW_TRAVEL_MM == G.layout.travel_x == 10.5 < min(closing, opening) - 0.3
    assert (23.0 - 2 * JAW_TRAVEL_MM, 23.0 + 2 * JAW_TRAVEL_MM) == (2.0, 44.0)   # the jaws' gap, closed and open


@pytest.mark.slow
def test_module_totals():
    shape = _module()
    assert gripper.totals(shape=shape) == {k: gripper.EXPECTED[k] for k in ("leaves", "solids", "solid_volume")}
    for body, want in gripper.EXPECTED["bodies"].items():
        assert gripper.totals(body, shape=shape) == want, body


# the one budget: the servo's vendor model is 41.0 long between posts 40.5 apart (a 0.25 press a side; the datasheet's
# 40.7, 0.1), and its spline is pressed into the pinion's bore - every other pair stays apart
BUDGET = {frozenset({"mg996r_servo:gripper", "gripper_base"}): 60.0, frozenset({"mg996r_servo:gripper", "gripper_pinion"}): 8.0}


def _module():
    from lib.models import raw

    return raw(gripper.gripper)


@pytest.mark.slow
def test_the_parts_fit():
    kids = list(_module().children)
    for i, a in enumerate(kids):
        for b in kids[i + 1:]:
            v = interference(a, b)
            assert v <= BUDGET.get(frozenset({a.label, b.label}), 0.01), (a.label, b.label, v)


@pytest.mark.slow
@pytest.mark.parametrize("travel", [-JAW_TRAVEL_MM, -5.0, 5.0, JAW_TRAVEL_MM])
def test_the_jaws_run_in_mesh_and_clear(travel):
    """Opened (+) or closed (-) by `travel` each: jaw a and rack a along -Y, b along +Y, the pinion turned by the rack's
    pitch over its 14 teeth - no rack or jaw meets the pinion, the base, the cage, the servo or the other side."""
    import build123d as bd

    kids = {c.label: c for c in _module().children}
    rad = travel / (G.pinion.teeth * G.rack.pitch / (2 * math.pi))   # the pinion turns travel / its pitch radius on the racks
    moved = {
        "gripper_rack:a": kids["gripper_rack:a"].moved(bd.Location((0.0, -travel, 0.0))),
        "gripper_jaw:a": kids["gripper_jaw:a"].moved(bd.Location((0.0, -travel, 0.0))),
        "gripper_rack:b": kids["gripper_rack:b"].moved(bd.Location((0.0, travel, 0.0))),
        "gripper_jaw:b": kids["gripper_jaw:b"].moved(bd.Location((0.0, travel, 0.0))),
    }
    pinions = [kids["gripper_pinion"].moved(bd.Location((0, 0, 0), (0, 0, s * math.degrees(rad)))) for s in (1.0, -1.0)]
    # the sign the pinion turns depends on which rack drives it: one of the two turns leaves both racks clear
    fixed = [kids[k] for k in ("gripper_base", "gripper_cage", "mg996r_servo:gripper", "gripper_j3_connector")]
    for name, part in moved.items():
        for other in fixed:
            assert interference(part, other) <= 0.01, (name, other.label, travel)
    assert interference(moved["gripper_jaw:a"], moved["gripper_jaw:b"]) <= 0.01
    assert interference(moved["gripper_rack:a"], moved["gripper_rack:b"]) <= 0.01
    assert any(all(interference(p, moved[r]) <= 0.05 for r in ("gripper_rack:a", "gripper_rack:b")) for p in pinions), travel


@pytest.mark.slow
def test_the_cage_meets_the_connector():
    """The cage's four screw holes on the connector's plate's (and their nuts' slots out to an edge); its cable's hole
    on the connector's bore; it seats on the plate's face."""
    from lib.gripper.body import connector_holes
    from tests import built

    cage = built.part("gripper_cage")
    z = layout.stations()
    mid = z["back_in"] + G.cage.back_t / 2.0
    for hx, hy in connector_holes():   # the hole, and solid plate beside it (away from the tab screws' access holes)
        assert not is_inside(cage, hx, hy, mid) and is_inside(cage, hx + (2.2 if hx > layout.x0() else -2.2), hy, z["back_in"] + 0.3)
    assert not is_inside(cage, layout.x0(), 0.0, mid)                       # the plug's hole on the axis
    assert cage.bounding_box().max.Z == pytest.approx(z["seat"])
    # the holes in the module frame land on the connector's: its plate's (x, z) -> the module's (-x, -z)
    from lib.wrist.params import DEFAULT as WRIST

    for (hx, hy), (x, zc, _) in zip(connector_holes(), WRIST.connector.holes, strict=True):
        assert layout.to_module((hx, hy, z["seat"])) == pytest.approx((-x, -zc, 0.0))
