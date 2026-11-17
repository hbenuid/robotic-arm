"""The wrist belt's tensioner (lib/forearm/params.py IdlerParams; cad/AGENTS.md "Belts"): the stock wrist belt is longer
than the open loop at WRIST_MOTOR_CENTRES, so an idler (gt2_idler_20t#3) on its back, on the run leaving the motor's
20T, IdlerParams.along from the motor, takes up the rest (lib/forearm/layout.py wrist_belt_loop, wrist_idler_xy). It
hangs under j2_link's web under a printed spacer (wrist_idler_spacer: the web's underside is the face j2_link prints
on), on an M5 (wrist_idler_screw) up through the idler, the spacer and the web's slot into a nut (wrist_idler_nut) in a
channel down from the web's top face: the slot and the channel along y between slot_y's ends, so the one idler takes
any belt from the open loop to STRETCH past the stock one. The axle, the nut and the spacer have no reference
(lib/reference.py NO_REFERENCE), so THIS file locks their numbers (pieces, the modelled volume and the bounding box,
measured on the build and written in). The idler keeps clear of the far strand and the pulleys anywhere in its slot; the
slot and the channel are probed on j2_link; the kit fits.
Every belt's own loop: tests/test_belts.py."""
import math

import pytest
from build123d import Location
from cad.build123d import closest_points

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
from lib.forearm import DEFAULT, LEGACY, WRIST_MOTOR_CENTRES, wrist_belt_loop, wrist_idler_spacer_h, wrist_idler_xy
from lib.geom import hex_circumdiameter
from tests import built
from tests.helpers import interference, is_inside

I, W, R_END = DEFAULT.idler, DEFAULT.web, DEFAULT.roll_end
XI = DEFAULT.motor_x - I.along                     # the idler's axis along the forearm (j2_link's x)
Z_IDLER = PARAMS.J2_BELT_Z - GT2_IDLER_WIDTH / 2.0   # its lower face: the screw's head's bearing face
# name -> (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCKS = {
    "wrist_idler_screw": (1, 641.783, (-4.25, -4.25, -5.0), (8.5, 8.5, 25.0)),
    "wrist_idler_nut": (1, 168.216, (-4.0, -4.618802, 0.0), (8.0, 9.237604, 4.7)),
    "wrist_idler_spacer": (1, 211.272, (-6.0, -6.0, 0.0), (12.0, 12.0, 3.2)),
}
FAR_CLEAR = 3.0    # the idler's flange off the far strand's teeth anywhere in its slot
STRETCH = 3.0      # the stock belt this far, at least, from either end of the band of belts the slot takes


def _loop_with_idler_at(y: float) -> list:
    """The wrist belt's loop with the idler's axis at j2_link's y (belt-frame y = -y)."""
    loop = wrist_belt_loop(DEFAULT, idler=False)
    loop.insert(1, (I.along, -y, -idler_pitch_r()))
    return loop


def _far_gap(y: float) -> float:
    """The idler's flange to the far strand's teeth (the run from the 90T back to the 20T), the idler at y."""
    (ax, ay), (bx, by), _ = belt_runs(wrist_belt_loop(DEFAULT, idler=False))[1]
    px, py = I.along, -y
    t = max(0.0, min(1.0, ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / ((bx - ax) ** 2 + (by - ay) ** 2)))
    d = math.hypot(px - ax - t * (bx - ax), py - ay - t * (by - ay))
    return d - GT2_IDLER_FLANGE_DIA / 2.0 - (GT2_BELT_THICKNESS - GT2_BELT_BACK)


def test_the_stock_belt_needs_its_idler():
    """The stock belt is longer than the open loop at the design centre distance; the idler on the run leaving the 20T,
    along from the motor, pushed in till the loop is the belt's length exactly - inside its slot."""
    loop, open_loop = wrist_belt_loop(), wrist_belt_loop(idler=False)
    assert R_END.wrist_belt in STANDARD_2GT_LENGTHS and WRIST_MOTOR_CENTRES == 80.0 and LEGACY.idler is None
    assert len(open_loop) == 2 and len(loop) == 3 and open_loop[1][0] == WRIST_MOTOR_CENTRES
    assert R_END.wrist_belt - belt_length(open_loop) > STRETCH
    assert belt_length(loop) == pytest.approx(R_END.wrist_belt, abs=1e-6)
    x, y, r = loop[1]
    assert (x, r) == (I.along, -idler_pitch_r()) and wrist_idler_xy() == (XI, -y)
    assert I.slot_y[0] < -y < I.slot_y[1]


def test_the_slot_takes_a_band_of_belts():
    """At the slot's outer end the idler clears the open loop's strand (it takes any belt down to the open loop, the
    belt goes on slack); at its inner end the belt that fits is the longest it takes; the stock belt STRETCH or more
    inside both."""
    (ax, ay), (bx, by), _ = belt_runs(wrist_belt_loop(idler=False))[0]
    strand = ay + (by - ay) * (I.along - ax) / (bx - ax)          # the open loop's strand at the idler's x (belt frame)
    assert -I.slot_y[1] + idler_pitch_r() < strand
    longest = belt_length(_loop_with_idler_at(I.slot_y[0]))
    assert longest - R_END.wrist_belt >= STRETCH and R_END.wrist_belt - belt_length(wrist_belt_loop(idler=False)) >= STRETCH
    assert longest == pytest.approx(283.15, abs=0.05)


@pytest.mark.parametrize("y", [I.slot_y[0], wrist_idler_xy()[1], I.slot_y[1]])
def test_the_idler_clears_the_far_strand_and_the_pulleys(y):
    """Anywhere in its slot: FAR_CLEAR off the far strand, well off the 20T and the 90T (their pitch circles; the parts
    themselves: test_the_kit_fits)."""
    assert _far_gap(y) >= FAR_CLEAR
    flange = GT2_IDLER_FLANGE_DIA / 2.0
    for cx, cy, r in wrist_belt_loop(idler=False):
        assert math.hypot(I.along - cx, -y - cy) - r - flange >= 5.0


def test_the_stations():
    """Along N: the idler level with the belt, the spacer from its top face to the web's underside, its land narrower
    than the idler's seat (the bearing's inner ring only); the screw's head under the idler, its tip at or past the nut's
    top on the channel's floor, under the web's top face, 3 of web under the nut. Across: the channel inside the web, off
    the motor's pilot and bolt holes."""
    assert Z_IDLER + GT2_IDLER_WIDTH / 2.0 == pytest.approx(PARAMS.J2_BELT_Z)
    assert wrist_idler_spacer_h() == pytest.approx(W.z0 - (Z_IDLER + GT2_IDLER_WIDTH)) and wrist_idler_spacer_h() > 2.0
    assert wrist_idler_spacer_h() > I.land_h and GT2_IDLER_BORE < I.slot_w < I.land_dia < min(GT2_IDLER_SEAT_DIA, I.riser_dia)
    tip, nut_top = Z_IDLER + I.screw_len, I.nut_floor + I.nut.h
    assert nut_top <= tip < W.z1 and I.nut_floor - W.z0 == pytest.approx(3.0)
    reach = hex_circumdiameter(I.nut.af) / 2.0 + I.nut_fit
    assert I.slot_y[1] + reach <= W.half_w - 2.0
    h, half = DEFAULT.holes, (I.nut.af + I.nut_fit) / 2.0
    assert DEFAULT.motor_x - h.bolt_half - h.bolt_dia / 2.0 - (XI + half) >= 2.0
    assert XI - half - (W.wrist_x + DEFAULT.boss.dia / 2.0) >= 15.0


def test_the_kit_is_parts_with_no_reference():
    for name, (pieces, volume, lo, size) in LOCKS.items():
        mod, shape = parts.load(name), parts.build(name)
        assert parts.GROUPS[name] == "joints" and name not in parts.unplaced() and len(shape.solids()) == pieces
        assert parts.bought(name) == (name != "wrist_idler_spacer")
        assert R.NO_REFERENCE[name] in (f"parts/joints/{name}.py:_envelope()", "lib/forearm/link.py:build_idler_spacer(DEFAULT)")
        if parts.bought(name):
            assert mod.PURCHASE_QTY == pieces
        assert R.solid_volume(shape) == pytest.approx(volume, abs=1e-3)
        bb = shape.bounding_box()
        assert tuple(bb.min) == pytest.approx(lo, abs=1e-6) and tuple(bb.size) == pytest.approx(size, abs=1e-6)
    assert parts.load("wrist_idler_screw").MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * shcs_volume(I.screw, I.screw_len))
    assert parts.load("wrist_idler_nut").MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * nut_volume(I.nut))


@pytest.mark.slow
def test_the_slot_and_the_channel_on_j2_link():
    """No rib under the web (it prints on its underside): the M5 slot through the web, the nut's channel (its flats +
    nut_fit across x) from nut_floor up through the top face - along y over the slot; the floor under the nut, solid
    past the channel's ends."""
    link = built.part("j2_link")
    mid, half = (W.z0 + I.nut_floor) / 2.0, (I.nut.af + I.nut_fit) / 2.0
    reach = hex_circumdiameter(I.nut.af) / 2.0 + I.nut_fit
    assert link.bounding_box().min.Z == pytest.approx(W.z0)
    for y in (I.slot_y[0], wrist_idler_xy()[1], I.slot_y[1]):
        assert not is_inside(link, XI, y, mid) and is_inside(link, XI + I.slot_w / 2.0 + 0.3, y, mid)       # the slot in the web
        assert not is_inside(link, XI + I.slot_w / 2.0 + 0.3, y, W.z0 - 0.5)                                # no rib under it
        assert not is_inside(link, XI + half - 0.2, y, I.nut_floor + 1.0) and is_inside(link, XI + half + 0.3, y, I.nut_floor + 1.0)
        assert is_inside(link, XI + I.slot_w / 2.0 + 0.3, y, I.nut_floor - 1.5)                              # the floor
    for y in (I.slot_y[0] - reach - 0.3, I.slot_y[1] + reach + 0.3):
        assert is_inside(link, XI, y, I.nut_floor + 1.0) and is_inside(link, XI, y, W.z0 + 0.5)


@pytest.mark.slow
def test_the_kit_fits():
    """In j2_link's frame: the idler on the belt, the spacer between the idler and the web, the nut on its channel's
    floor - touching, nothing overlapping (the wrist motor's screws' heads, flush in the web's underside, too); the screw
    clear in its slot; the idler 5 mm or more off the 20T and the 90T at its place and at both ends of its slot."""
    into = "j2_link#1"
    link, belt = built.part("j2_link"), built.placed("wrist_belt#1", into=into)
    idler, spacer, screw, nut = (built.placed(k, into=into) for k in
                                 ("gt2_idler_20t#3", "wrist_idler_spacer#1", "wrist_idler_screw#1", "wrist_idler_nut#1"))
    pulleys = [built.placed(k, into=into) for k in ("gt2_pulley_20t#4", "gt2_pulley_90t#4")]
    near = [link, belt, *pulleys, built.placed("nema17_40mm#3", into=into), built.placed("wrist_motor_screws#1", into=into)]
    kit = [idler, spacer, screw, nut]
    for part in kit:
        for other in near + kit:
            if other is not part:
                assert interference(part, other) < 0.5
    assert closest_points(belt, idler).distance < 0.01 and closest_points(spacer, idler).distance < 0.01
    assert closest_points(spacer, link).distance < 0.01 and closest_points(nut, link).distance < 0.01
    assert closest_points(screw, link).distance >= (I.slot_w - I.screw.d) / 2.0 - 0.01
    y0 = wrist_idler_xy()[1]
    for y in I.slot_y:
        moved = idler.moved(Location((0.0, y - y0, 0.0)))
        assert all(closest_points(moved, p).distance >= 5.0 for p in pulleys)
