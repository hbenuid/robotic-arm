"""The elbow belt's tensioner (lib/upper_arm/params.py ElbowIdlerParams; cad/AGENTS.md "Belts"): the stock elbow belt is
longer than the open loop at ELBOW_MOTOR_CENTRES, so an idler (gt2_idler_20t) on its back, on the run from the 108T back
to the motor's 20T, ElbowIdlerParams.along from the motor (beside the stepped slot's line), takes up the rest (lib/upper_arm/layout.py elbow_belt_loop,
elbow_idler_z). It hangs under j1_link's slab under a printed spacer (elbow_idler_spacer: the slab's underside is the face
the part prints on), on an M5 (elbow_idler_screw) up through the idler, the spacer and the slot in the slab's floor into
a nut (elbow_idler_nut) in a slot down from the slab's top face: both slots along Z between slot_z's ends, so the one
idler takes any belt from the open loop to the longest that leaves it FAR_CLEAR off the far strand - the stock belt in
the middle. The axle, the nut and the spacer have no reference (lib/reference.py NO_REFERENCE), so THIS file locks
their numbers (pieces, the modelled volume and the bounding box, measured on the build and written in). The idler keeps
clear of the far strand, the 108T and the 20T at both ends of its slot; the slots are probed on j1_link; the kit fits."""
import math

import pytest
from build123d import Location
from cadgen.geometry import closest_points

import parts
from lib import params as PARAMS
from lib import reference as R
from lib.belts import (
    GT2_BELT_BACK,
    GT2_BELT_THICKNESS,
    GT2_IDLER_BORE,
    GT2_IDLER_FLANGE_DIA,
    GT2_IDLER_SEAT_DIA,
    GT2_IDLER_WIDTH,
    STANDARD_2GT_LENGTHS,
    belt_length,
    belt_runs,
    idler_pitch_r,
)
from lib.fasteners import nut_volume, shcs_volume
from lib.geom import hex_circumdiameter
from lib.upper_arm import DEFAULT, LEGACY, arm_slide, elbow_belt_loop, elbow_idler_x, elbow_idler_z
from lib.upper_arm.params import ELBOW_BELT, ELBOW_MOTOR_CENTRES
from tests import built
from tests.helpers import interference, is_inside

I, SL = DEFAULT.idler, DEFAULT.slots
X = elbow_idler_x()                               # the idler's axis along the link, beside the stepped slot's line
ALONG = I.along                                   # ... from the motor's, along the belt
TOP = DEFAULT.elbow.relief_y + arm_slide(DEFAULT)  # the slab's top face (the roll frame turns over it)
# name -> (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCKS = {
    "elbow_idler_screw": (1, 739.958, (-4.25, -4.25, -5.0), (8.5, 8.5, 30.0)),
    "elbow_idler_nut": (1, 168.216, (-4.0, -4.618802, 0.0), (8.0, 9.237604, 4.7)),
    "elbow_idler_spacer": (1, 211.273, (-6.0, -6.0, 0.0), (12.0, 12.0, 3.2)),
}
FAR_CLEAR = 3.0    # the idler's flange off the far strand's teeth at the slot's inner end
PULLEY_CLEAR_AT_ENDS = 4.5   # the idler's flange off the pulleys' flanges at the slot's ends (5.0 at its place)
STRETCH = 5.0      # the stock belt this far, at least, from either end of the band of belts the slot takes


def _loop_with_idler_at(z: float) -> list:
    """The elbow belt's loop with the idler's axis at z (part z = belt-frame y)."""
    loop = elbow_belt_loop(DEFAULT, idler=False)
    loop.insert(I.run + 1, (ALONG, z, -idler_pitch_r()))
    return loop


def _far_gap(z: float) -> float:
    """The idler's flange to the far strand's teeth (the other run, the 20T to the 108T), the idler at z."""
    (ax, ay), (bx, by), _ = belt_runs(_loop_with_idler_at(z))[0]
    t = max(0.0, min(1.0, ((ALONG - ax) * (bx - ax) + (z - ay) * (by - ay)) / ((bx - ax) ** 2 + (by - ay) ** 2)))
    d = math.hypot(ALONG - ax - t * (bx - ax), z - ay - t * (by - ay))
    return d - GT2_IDLER_FLANGE_DIA / 2.0 - (GT2_BELT_THICKNESS - GT2_BELT_BACK)


def test_the_stock_belt_needs_its_idler():
    """The stock belt is longer than the open loop at the design centre distance; the idler on the 108T -> 20T run,
    `along` from the motor (1.9 short of the stepped slot's line), pushed in till the loop is the belt's length exactly -
    inside its slot."""
    assert X == pytest.approx(DEFAULT.pad.x + ALONG, abs=1e-9) and SL.stepped_x - X == pytest.approx(1.9, abs=1e-9)
    loop, open_loop = elbow_belt_loop(), elbow_belt_loop(idler=False)
    assert ELBOW_BELT in STANDARD_2GT_LENGTHS and ELBOW_MOTOR_CENTRES == 78.0 and LEGACY.idler is None
    assert len(open_loop) == 2 and len(loop) == 3 and open_loop[1][0] == ELBOW_MOTOR_CENTRES
    assert ELBOW_BELT - belt_length(open_loop) > STRETCH
    assert belt_length(loop) == pytest.approx(ELBOW_BELT, abs=1e-6)
    x, z, r = loop[I.run + 1]
    assert (x, r) == (ALONG, -idler_pitch_r()) and z == elbow_idler_z() and I.slot_z[0] < z < I.slot_z[1]


def test_the_slot_takes_a_band_of_belts():
    """At the slot's outer end the idler clears the open loop's strand (it takes any belt down to the open loop); at
    its inner end it keeps FAR_CLEAR off the far strand, and the belt that fits there is the longest it takes; the stock
    belt STRETCH or more inside both."""
    (ax, ay), (bx, by), _ = belt_runs(elbow_belt_loop(idler=False))[I.run]
    strand = ay + (by - ay) * (ALONG - ax) / (bx - ax)              # the open loop's strand at the idler's x
    assert I.slot_z[1] - idler_pitch_r() > strand
    assert _far_gap(I.slot_z[0]) >= FAR_CLEAR
    longest = belt_length(_loop_with_idler_at(I.slot_z[0]))
    assert longest - ELBOW_BELT >= STRETCH and ELBOW_BELT - belt_length(elbow_belt_loop(idler=False)) >= STRETCH
    assert longest == pytest.approx(315.7, abs=0.05)


@pytest.mark.parametrize("z", [I.slot_z[0], elbow_idler_z(), I.slot_z[1]])
def test_the_idler_clears_the_far_strand_and_the_pulleys(z):
    """Anywhere in its slot: FAR_CLEAR off the far strand (more off its place), well off the 20T and the 108T (their
    pitch circles; the parts themselves: test_the_kit_fits)."""
    assert _far_gap(z) >= FAR_CLEAR
    flange = GT2_IDLER_FLANGE_DIA / 2.0
    for cx, cy, r in elbow_belt_loop(idler=False):
        assert math.hypot(ALONG - cx, z - cy) - r - flange >= 5.0


def test_the_stations():
    """Along N: the idler level with the belt, the spacer from its upper face to the slab's underside, its land narrower
    than the idler's seat (the bearing's inner ring only); the screw's head under the idler, its tip 2 pitches past the
    nut on its slot's floor, the nut under the top face; across: the nut's slot off the motor's hole, inside the slab."""
    assert PARAMS.J1_IDLER_Y + GT2_IDLER_WIDTH / 2.0 == pytest.approx(PARAMS.J1_BELT_Y)
    assert PARAMS.J1_SLAB_UNDERSIDE_Y == pytest.approx(DEFAULT.elbow.y0 + arm_slide(DEFAULT))
    assert PARAMS.J1_IDLER_SPACER_H == pytest.approx(PARAMS.J1_SLAB_UNDERSIDE_Y - PARAMS.J1_IDLER_Y - GT2_IDLER_WIDTH)
    assert PARAMS.J1_IDLER_SPACER_H > I.land_h and GT2_IDLER_BORE < I.slot_w < I.land_dia < min(GT2_IDLER_SEAT_DIA, I.riser_dia)
    tip, nut_top = PARAMS.J1_IDLER_Y + I.screw_len, PARAMS.J1_IDLER_LEDGE_Y + I.nut.h
    assert tip - nut_top >= 2 * PARAMS.M5_PITCH - 1e-9 and TOP - nut_top >= 0.5 and I.floor >= 3.0
    hole_edge = DEFAULT.pad.x + DEFAULT.arm.motor_hole_half
    assert X - (I.nut.af + I.nut_fit) / 2.0 - hole_edge >= 3.0
    assert I.slot_z[1] + hex_circumdiameter(I.nut.af) / 2.0 + I.nut_fit < DEFAULT.slab.r - 10.0


def test_the_kit_is_parts_with_no_reference():
    for name, (pieces, volume, lo, size) in LOCKS.items():
        mod, shape = parts.load(name), parts.build(name)
        assert parts.GROUPS[name] == "joints" and name not in parts.unplaced() and len(shape.solids()) == pieces
        assert parts.bought(name) == (name != "elbow_idler_spacer")
        assert R.NO_REFERENCE[name].startswith(("parts/joints/", "lib/upper_arm/link.py"))
        assert R.solid_volume(shape) == pytest.approx(volume, abs=1e-3)
        bb = shape.bounding_box()
        assert tuple(bb.min) == pytest.approx(lo, abs=1e-6) and tuple(bb.size) == pytest.approx(size, abs=1e-6)
        if parts.bought(name):
            assert mod.PURCHASE_QTY == pieces
    assert parts.load("elbow_idler_screw").MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * shcs_volume(I.screw, I.screw_len))
    assert parts.load("elbow_idler_nut").MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * nut_volume(I.nut))


@pytest.mark.slow
def test_the_slots_on_j1_link():
    """The axle's slot through the floor and the nut's slot from the floor up through the top face, along Z over the
    travel; the floor under the nut, the walls beside each slot, solid past the nut slot's ends."""
    link = built.part("j1_link")
    u, ledge = PARAMS.J1_SLAB_UNDERSIDE_Y, PARAMS.J1_IDLER_LEDGE_Y
    half_nut, reach = (I.nut.af + I.nut_fit) / 2.0, hex_circumdiameter(I.nut.af) / 2.0 + I.nut_fit
    for z in (I.slot_z[0], elbow_idler_z(), I.slot_z[1]):
        for y in (u + 0.5, ledge - 0.5):                                       # the axle's slot through the floor
            assert not is_inside(link, X, y, z) and is_inside(link, X + I.slot_w / 2.0 + 0.3, y, z)
        for y in (ledge + 0.5, TOP - 0.5):                                     # the nut's slot, open up through the top
            assert not is_inside(link, X + half_nut - 0.2, y, z) and is_inside(link, X + half_nut + 0.3, y, z)
    for y in (u + 0.5, ledge + 2.0, TOP - 0.5):                                # solid past both ends
        assert is_inside(link, X, y, I.slot_z[0] - reach - 0.3) and is_inside(link, X, y, I.slot_z[1] + reach + 0.3)


@pytest.mark.slow
def test_the_kit_fits():
    """In j1_link's frame: the idler on the belt, the spacer between the idler and the slab, the nut on its slot's
    floor - touching, nothing overlapping; the screw clear in its slot; the idler 5 mm or more off the 20T and the 108T
    at its place, PULLEY_CLEAR_AT_ENDS at both ends of its slot (the inner end, the longest belt it takes, 4.74 off the
    108T's flange; its nearest, 4.58, half way along at z 0)."""
    into = "j1_link#1"
    link, belt = built.part("j1_link"), built.placed("elbow_belt#1", into=into)
    idler, spacer, screw, nut = (built.placed(f"{k}#1", into=into) for k in
                                 ("gt2_idler_20t", "elbow_idler_spacer", "elbow_idler_screw", "elbow_idler_nut"))
    near = [link, belt, *(built.placed(k, into=into) for k in ("gt2_pulley_20t#2", "gt2_pulley_108t#1", "nema17_40mm#2"))]
    kit = [idler, spacer, screw, nut]
    for part in kit:
        for other in near + kit:
            if other is not part:
                assert interference(part, other) < 0.5
    assert closest_points(belt, idler).distance < 0.01 and closest_points(spacer, idler).distance < 0.01
    assert closest_points(spacer, link).distance < 0.01 and closest_points(nut, link).distance < 0.01
    assert closest_points(screw, link).distance >= (I.slot_w - I.screw.d) / 2.0 - 0.01
    pulleys = [built.placed(k, into=into) for k in ("gt2_pulley_20t#2", "gt2_pulley_108t#1")]
    assert all(closest_points(idler, p).distance >= 5.0 for p in pulleys)
    for z in (I.slot_z[0], I.slot_z[1]):
        moved = idler.moved(Location((0.0, 0.0, z - elbow_idler_z())))
        assert all(closest_points(moved, p).distance >= PULLEY_CLEAR_AT_ENDS for p in pulleys)
