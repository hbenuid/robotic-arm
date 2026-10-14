"""The base_yaw belt's tensioner (lib/base/params.py YawIdlerParams; cad/AGENTS.md "Belts"): the stock base_yaw belt is
longer than the open loop at YAW_MOTOR_X, so an idler (gt2_idler_20t) on its back, on the run from the 120T back to the
motor's 20T, on the belt's long open run over the motor mount's plate, takes up the rest (lib/base/layout.py
yaw_belt_loop, yaw_idler_xz). It stands on the plate on a printed spacer (yaw_idler_spacer: the mount prints plate-down),
on an M5 (yaw_idler_screw) down through the idler, the spacer and a slot through the plate into a nut (yaw_idler_nut)
between two rails under the plate: the slot and the nut's slot run across the belt between slot_y's ends, so the one
idler takes any belt from the open loop to the longest that leaves it FAR_CLEAR off the far strand. The axle, the nut
and the spacer have no reference (lib/reference.py NO_REFERENCE), so THIS file locks their numbers (pieces, the modelled
volume and the bounding box, measured on the build and written in). The idler keeps clear of the far strand, the 20T
and the 120T at both ends of its slot; the seat is probed on the mount; the kit fits."""
import math

import pytest
from build123d import Location
from cadgen.geometry import closest_points

import parts
from lib import params as PARAMS
from lib import reference as R
from lib.base import (
    DEFAULT,
    LEGACY,
    mount_inner_half,
    yaw_belt_frame,
    yaw_belt_loop,
    yaw_belt_point,
    yaw_idler_slot,
    yaw_idler_spacer_h,
    yaw_idler_xz,
)
from lib.base.params import YAW_BELT, YAW_BELT_Y, YAW_MOTOR_X
from lib.belts import (
    GT2_BELT_BACK,
    GT2_BELT_THICKNESS,
    GT2_IDLER_BORE,
    GT2_IDLER_FLANGE_DIA,
    GT2_IDLER_SEAT_DIA,
    GT2_IDLER_WIDTH,
    GT2_PULLEY_20T_TEETH,
    GT2_PULLEY_120T_TEETH,
    STANDARD_2GT_LENGTHS,
    belt_length,
    belt_runs,
    centre_distance,
    idler_pitch_r,
)
from lib.datum import to_location
from lib.fasteners import nut_volume, shcs_volume
from lib.geom import hex_circumdiameter
from lib.motors import NEMA17_FACE
from tests import built
from tests.helpers import interference, is_inside

I, PL = DEFAULT.idler, DEFAULT.plate
C = math.hypot(*DEFAULT.motor.centre)            # the motor's axis to the base_yaw axis, along the belt
ALONG = C - I.x                                  # the idler's axis from the motor's, along the belt
# name -> (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCKS = {
    "yaw_idler_screw": (1, 739.958, (-4.25, -4.25, -5.0), (8.5, 8.5, 30.0)),
    "yaw_idler_nut": (1, 168.216, (-4.0, -4.618802, 0.0), (8.0, 9.237604, 4.7)),
    "yaw_idler_spacer": (1, 309.545, (-6.0, -6.0, 0.0), (12.0, 12.0, 4.3)),
}
FAR_CLEAR = 3.0    # the idler's flange off the far strand's teeth at the slot's inner end
STRETCH = 5.0      # the stock belt this far, at least, from either end of the band of belts the slot takes
HALF_NUT = (I.nut.af + I.nut_fit) / 2.0                          # the nut's slot, half across
REACH = hex_circumdiameter(I.nut.af) / 2.0 + I.nut_fit            # ... and past each end of the axle's travel


def _loop_with_idler_at(y: float) -> list:
    """The base_yaw belt's loop with the idler's axis at y across the belt (its frame's y)."""
    loop = yaw_belt_loop(DEFAULT, idler=False)
    loop.insert(I.run + 1, (ALONG, y, -idler_pitch_r()))
    return loop


def _far_gap(y: float) -> float:
    """The idler's flange to the far strand's teeth (the other run, the 20T to the 120T), the idler at y."""
    (ax, ay), (bx, by), _ = belt_runs(_loop_with_idler_at(y))[0]
    t = max(0.0, min(1.0, ((ALONG - ax) * (bx - ax) + (y - ay) * (by - ay)) / ((bx - ax) ** 2 + (by - ay) ** 2)))
    d = math.hypot(ALONG - ax - t * (bx - ax), y - ay - t * (by - ay))
    return d - GT2_IDLER_FLANGE_DIA / 2.0 - (GT2_BELT_THICKNESS - GT2_BELT_BACK)


def _at(along: float, y: float) -> tuple[float, float, float]:
    """A point `along` the belt from the motor's axis, `y` across it, at the belt's height - in the base's frame."""
    x, z = yaw_belt_point(along, y)
    return (x, YAW_BELT_Y, z)


def test_the_stock_belt_needs_its_idler():
    """The stock belt is longer than the open loop at the motor's design centre - short of the belt's own centre distance;
    the idler on the 120T -> 20T run, x out from the axis, pushed in till the loop is the belt's length exactly - inside
    its slot."""
    loop, open_loop = yaw_belt_loop(), yaw_belt_loop(idler=False)
    assert YAW_BELT in STANDARD_2GT_LENGTHS and YAW_MOTOR_X == 122.0 and LEGACY.idler is None
    assert YAW_MOTOR_X < centre_distance(YAW_BELT, GT2_PULLEY_120T_TEETH, GT2_PULLEY_20T_TEETH)
    assert len(open_loop) == 2 and len(loop) == 3 and open_loop[1][0] == pytest.approx(C)
    assert YAW_BELT - belt_length(open_loop) > STRETCH
    assert belt_length(loop) == pytest.approx(YAW_BELT, abs=1e-6)
    x, y, r = loop[I.run + 1]
    assert (x, r) == (ALONG, -idler_pitch_r()) and I.slot_y[0] < y < I.slot_y[1]


def test_the_slot_takes_a_band_of_belts():
    """At the slot's outer end the idler clears the open loop's strand (it takes any belt down to the open loop); at
    its inner end it keeps FAR_CLEAR off the far strand, and the belt that fits there is the longest it takes; the stock
    belt STRETCH or more inside both."""
    (ax, ay), (bx, by), _ = belt_runs(yaw_belt_loop(idler=False))[I.run]
    strand = ay + (by - ay) * (ALONG - ax) / (bx - ax)              # the open loop's strand at the idler's x
    assert I.slot_y[1] - idler_pitch_r() > strand
    assert _far_gap(I.slot_y[0]) >= FAR_CLEAR
    longest = belt_length(_loop_with_idler_at(I.slot_y[0]))
    assert longest - YAW_BELT >= STRETCH and YAW_BELT - belt_length(yaw_belt_loop(idler=False)) >= STRETCH
    assert longest == pytest.approx(405.3, abs=0.05)


@pytest.mark.parametrize("y", [I.slot_y[0], yaw_belt_loop()[I.run + 1][1], I.slot_y[1]])
def test_the_idler_clears_the_far_strand_and_the_pulleys(y):
    """Anywhere in its slot: FAR_CLEAR off the far strand (more off its place), well off the 20T and the 120T (their
    pitch circles; the parts themselves: test_the_kit_fits)."""
    assert _far_gap(y) >= FAR_CLEAR
    flange = GT2_IDLER_FLANGE_DIA / 2.0
    for cx, cy, r in yaw_belt_loop(idler=False):
        assert math.hypot(ALONG - cx, y - cy) - r - flange >= 5.0


def test_the_slot_runs_across_the_belt():
    """The belt frame's points in the base's (x, z) as yaw_belt_frame() places them; the slot's ends ALONG the belt,
    across it at slot_y; the idler's axis on the slot's line between them, x out from the base_yaw axis."""
    frame = to_location(yaw_belt_frame())
    for along, y in ((0.0, 0.0), (ALONG, I.slot_y[0]), (ALONG, I.slot_y[1]), (C, 10.0)):
        p = (frame * Location((along, y, 0.0))).position
        assert yaw_belt_point(along, y) == pytest.approx((p.X, p.Z), abs=1e-6) and p.Y == pytest.approx(YAW_BELT_Y)   # the frame's angle: 6 places
    (x0, z0), (x1, z1) = yaw_idler_slot()
    assert (x0, z0) == pytest.approx(yaw_belt_point(ALONG, I.slot_y[0])) and (x1, z1) == pytest.approx(yaw_belt_point(ALONG, I.slot_y[1]))
    x, z = yaw_idler_xz()
    assert abs((x1 - x0) * (z - z0) - (z1 - z0) * (x - x0)) / math.dist((x0, z0), (x1, z1)) < 1e-9
    assert min(z0, z1) < z < max(z0, z1) and x == pytest.approx(I.x, abs=0.01)


def test_the_stations():
    """Along Y: the idler level with the belt, the spacer from the plate's top to its lower face, its land narrower than
    the idler's seat (the bearing's inner ring only); the screw's head on the idler's top, its tip 2 pitches past the nut
    on the plate's underside, the nut inside its rails; across: the nut's rails inside the mount's walls, past its ears
    and short of the motor's face."""
    assert PARAMS.YAW_IDLER_Y + GT2_IDLER_WIDTH / 2.0 == pytest.approx(YAW_BELT_Y)
    assert PARAMS.YAW_IDLER_SPACER_H == pytest.approx(yaw_idler_spacer_h()) == pytest.approx(PARAMS.YAW_IDLER_Y - PL.y[1])
    assert PARAMS.YAW_IDLER_SPACER_H > I.land_h and GT2_IDLER_BORE < I.slot_w < I.land_dia < min(GT2_IDLER_SEAT_DIA, I.riser_dia)
    tip, nut_low = PARAMS.YAW_IDLER_Y + GT2_IDLER_WIDTH - I.screw_len, PARAMS.YAW_IDLER_NUT_Y - I.nut.h
    assert PARAMS.YAW_IDLER_NUT_Y == PL.y[0] and nut_low - tip >= 2 * PARAMS.M5_PITCH - 1e-9 and I.rail_h >= I.nut.h
    (_, za), (_, zb) = yaw_idler_slot()
    assert max(abs(za), abs(zb)) + REACH < mount_inner_half() and I.rail_wall >= 2.0
    near, far = I.x - HALF_NUT - I.rail_wall, I.x + HALF_NUT + I.rail_wall
    assert near - (DEFAULT.joint.split_x + DEFAULT.mount.ear_t) >= 10.0
    assert DEFAULT.motor.centre[0] - NEMA17_FACE / 2.0 - far >= 5.0


def test_the_kit_is_parts_with_no_reference():
    for name, (pieces, volume, lo, size) in LOCKS.items():
        mod, shape = parts.load(name), parts.build(name)
        assert parts.GROUPS[name] == "base" and name not in parts.unplaced() and len(shape.solids()) == pieces
        assert parts.bought(name) == (name != "yaw_idler_spacer")
        assert R.NO_REFERENCE[name].startswith(("parts/base/", "lib/base/body.py"))
        assert R.solid_volume(shape) == pytest.approx(volume, abs=1e-3)
        bb = shape.bounding_box()
        assert tuple(bb.min) == pytest.approx(lo, abs=1e-6) and tuple(bb.size) == pytest.approx(size, abs=1e-6)
        if parts.bought(name):
            assert mod.PURCHASE_QTY == pieces
    assert parts.load("yaw_idler_screw").MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * shcs_volume(I.screw, I.screw_len))
    assert parts.load("yaw_idler_nut").MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * nut_volume(I.nut))


@pytest.mark.slow
def test_the_seat_on_the_mount():
    """The axle's slot through the plate and the nut's slot between the rails under it, across the belt over the
    travel; the plate whole over the nut's slot, the rails beside it, solid past both its ends."""
    mount = built.part("base_motor_mount")
    plate, rails = (PL.y[0] + PL.y[1]) / 2.0, PL.y[0] - I.rail_h / 2.0
    for y in (I.slot_y[0], yaw_belt_loop()[I.run + 1][1], I.slot_y[1]):
        for dy in (plate, rails):                                              # the axle's slot, through both
            x, _, z = _at(ALONG, y)
            assert not is_inside(mount, x, dy, z)
        for side in (1, -1):
            x, _, z = _at(ALONG + side * (I.slot_w / 2.0 + 0.3), y)            # the plate beside the axle's slot
            assert is_inside(mount, x, plate, z)
            x, _, z = _at(ALONG + side * (HALF_NUT - 0.2), y)                  # the nut's slot under the plate ...
            assert not is_inside(mount, x, rails, z) and is_inside(mount, x, PL.y[0] + 0.5, z)
            x, _, z = _at(ALONG + side * (HALF_NUT + 0.3), y)                  # ... between the rails
            assert is_inside(mount, x, rails, z)
    for y in (I.slot_y[0] - REACH - 0.3, I.slot_y[1] + REACH + 0.3):          # the rails' closed ends
        x, _, z = _at(ALONG, y)
        assert is_inside(mount, x, rails, z)


@pytest.mark.slow
def test_the_kit_fits():
    """In the base's frame: the idler on the belt, the spacer between the idler and the plate, the screw's head on the
    idler, the nut on the plate's underside between the rails - touching, nothing overlapping; the screw clear in its
    slot; the idler 5 mm or more off the 20T and the 120T at its place and at both ends of its slot."""
    into = "base_motor_mount#1"
    mount, belt = built.part("base_motor_mount"), built.placed("yaw_belt#1", into=into)
    idler, spacer, screw, nut = (built.placed(k, into=into) for k in
                                 ("gt2_idler_20t#2", "yaw_idler_spacer#1", "yaw_idler_screw#1", "yaw_idler_nut#1"))
    pulleys = [built.placed(k, into=into) for k in ("gt2_pulley_20t#3", "gt2_pulley_120t#1")]
    near = [mount, belt, built.placed("base#1", into=into), built.placed("nema17_48mm#1", into=into), *pulleys]
    kit = [idler, spacer, screw, nut]
    for part in kit:
        for other in near + kit:
            if other is not part:
                assert interference(part, other) < 0.5
    assert closest_points(belt, idler).distance < 0.01 and closest_points(spacer, idler).distance < 0.01
    assert closest_points(screw, idler).distance < 0.01
    assert closest_points(spacer, mount).distance < 0.01 and closest_points(nut, mount).distance < 0.01
    assert closest_points(screw, mount).distance >= (I.slot_w - I.screw.d) / 2.0 - 0.01
    x, z = yaw_idler_xz()
    for y in I.slot_y:
        ex, _, ez = _at(ALONG, y)
        moved = idler.moved(Location((ex - x, 0.0, ez - z)))
        assert all(closest_points(moved, p).distance >= 5.0 for p in pulleys)
