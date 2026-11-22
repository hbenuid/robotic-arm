"""The roll belt's tensioner (lib/forearm/params.py RollIdlerParams; cad/AGENTS.md "Belts"): the stock roll belt is
longer than the open loop at the drive's design centre distance, so an idler (gt2_idler_20t) on its back, on the run
back from the ring, takes up the rest (lib/forearm/layout.py roll_belt_loop, roll_idler_xy). It stands on a printed
spacer (roll_idler_spacer: the frame prints front face down, so no riser grows from the face) - its riser on the roll
frame's front face, its land under the idler's bearing inner ring - on an M5 (roll_idler_screw) down through the
spacer and the plate's slot into a nut (roll_idler_nut) in a channel under the face, closed at both ends, the nut in
from the motor's pocket - its far face set back to the channel over its upper part, the way open from below: the slot
and the channel along module X, from its slack end out of the loop to its tight end in it. The axle and the nut are
purchased parts and the spacer a printed one, all with no reference (lib/reference.py NO_REFERENCE), so THIS file locks
their numbers (pieces, the modelled volume and the bounding box, measured on the build and written in). The idler keeps
clear of the belt's far run, the ring's flange and the 20T's hub at its place and at both of its slot's ends, the belt
slack at the slack end; the slot, the channel and the nut's way in are probed on the frame, nothing on its face; the kit
fits in the module."""
import math

import pytest
from cad.build123d import closest_points

import parts
from assemblies import forearm_roll_drive as M
from lib import params as PARAMS
from lib import reference as R
from lib.belts import (
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
from lib.forearm import (
    DEFAULT,
    motor_pocket,
    roll_belt_loop,
    roll_idler_setback,
    roll_idler_spacer_h,
    roll_idler_xy,
    stack_positions,
)
from lib.forearm.roll import back_point
from lib.style import EDGE_CHAMFER
from tests import built
from tests.helpers import interference, is_inside

D, I, S = DEFAULT.drive, DEFAULT.drive.idler, stack_positions(DEFAULT)
SPACER = "roll_idler_spacer"
# name -> (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCKS = {
    "roll_idler_screw": (1, 739.958, (-4.25, -4.25, -5.0), (8.5, 8.5, 30.0)),
    "roll_idler_nut": (1, 168.216, (-4.618802, -4.0, 0.0), (9.237604, 8.0, 4.7)),
    SPACER: (1, 367.615, (-6.0, -6.0, 0.0), (12.0, 12.0, 4.95)),
}
CLEAR = 3.0        # the idler's flange off the belt's far run and the ring's flange, at its place and at its slot's deep end
HUB_R = 8.0        # the vendor 20T's hub (its flange side), the idler's flange in its plane at the belt's lower edge
RING_FLANGE_R = D.ring_flange_dia / 2.0


def _seg_dist(p, a, b) -> float:
    """Point p to the segment a-b, in the belt's plane."""
    (ax, ay), (bx, by), (px, py) = a, b, p
    t = max(0.0, min(1.0, ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / ((bx - ax) ** 2 + (by - ay) ** 2)))
    return math.hypot(px - (ax + t * (bx - ax)), py - (ay + t * (by - ay)))


def _clearances(push: float = 0.0) -> dict:
    """The idler's flange to the belt's far run (the loop's first run, the 20T to the ring), to the ring's flange and to
    the 20T's hub - the idler `push` further into the loop than its place (belt frame)."""
    loop = roll_belt_loop()
    x, y, _ = loop[2]
    y -= push                                 # into the loop: toward the far run (the loop's -y side)
    a, b, _ = belt_runs(loop)[0]
    flange = GT2_IDLER_FLANGE_DIA / 2.0
    return {"far run": _seg_dist((x, y), a, b) - flange,
            "ring": math.hypot(x - D.centre_distance, y) - RING_FLANGE_R - flange,
            "20T hub": math.hypot(x, y) - HUB_R - flange}


def test_the_stock_belt_needs_its_idler():
    """The stock roll belt is longer than the open loop at the design centre distance; the idler on the run back from
    the ring, RollIdlerParams.along from the motor, pushed in till the loop is the belt's length exactly."""
    loop, open_loop = roll_belt_loop(), roll_belt_loop(idler=False)
    assert D.roll_belt in STANDARD_2GT_LENGTHS and len(open_loop) == 2 and len(loop) == 3
    assert D.roll_belt - belt_length(open_loop) > 5.0
    assert belt_length(loop) == pytest.approx(D.roll_belt, abs=1e-6)
    x, y, r = loop[2]
    assert (x, r) == (I.along, -idler_pitch_r()) and abs(y) < belt_length(open_loop)
    ix, iy = roll_idler_xy()
    assert (ix, iy) == pytest.approx((S["x_motor"] - y, S["y_motor"] + x), abs=1e-9)


@pytest.mark.parametrize("push", [0.0, *I.slot])
def test_the_idler_clears_the_far_run_the_ring_and_the_20t(push):
    """At its place and at its slot's ends (the tight end further into the loop, the slack end out of it): CLEAR off
    the belt's far run and the ring's flange, which turn in its plane; well off the 20T's hub."""
    got = _clearances(push)
    assert got["far run"] >= CLEAR and got["ring"] >= CLEAR and got["20T hub"] >= 10.0, got


def test_the_slot_takes_the_belt_on_and_tightens_it():
    """The slack end backs the idler out of the loop far enough that the stock belt goes on 8 mm or more slack; the tight
    end pushes it in 3 mm or more past its place."""
    out, inn = I.slot
    loop = roll_belt_loop()
    x, y, r = loop[2]
    slack = D.roll_belt - belt_length([loop[0], loop[1], (x, y - out, r)])
    assert out < 0.0 < inn and slack >= 8.0 and inn >= 3.0


def test_the_spacer_the_screw_and_the_nut():
    """The idler level with the belt on the spacer's land - the land narrower than the idler's seat (its bearing's
    inner ring only), the spacer from the frame's front face up to the idler, its riser 3 mm or more round the axle and
    on the face's flat (inside the facets) anywhere in the slot; the screw's head on the idler's top, the shank through
    the spacer and the roof into the nut, its tip 2 pitches or more past the nut and inside its hole, its hole clear of
    the shaft's bay; the nut's bearing face on the channel's roof, the channel 2.5 or more off the motor pocket's box,
    under the front face, the nut's way in from the pocket starting entry inside its top edge, over the motor's face;
    the pocket's far face set back to the channel's near side from the motor's face down past the pocket's middle, then
    45 deg back to the face - its lower part, by the back panel's opening, where it was (tests/forearm/
    test_roll_back_panel.py keeps the wall there)."""
    assert S["z_idler"] + GT2_IDLER_WIDTH / 2.0 == pytest.approx(S["z_ring_mid"])
    assert roll_idler_spacer_h() == pytest.approx(S["z_idler"] - S["z_front"]) and roll_idler_spacer_h() > I.land_h + 2.0
    assert GT2_IDLER_BORE < I.slot_w < I.land_dia < min(GT2_IDLER_SEAT_DIA, I.riser_dia)
    assert (I.riser_dia - I.slot_w) / 2.0 >= 3.0
    ix, iy = roll_idler_xy()
    reach = max(abs(ix + end) for end in I.slot)                       # the screw's farthest from the frame's middle
    assert reach + I.riser_dia / 2.0 <= D.block_x[1] - D.front_facet - 2.0
    tip = S["z_idler_top"] - I.screw_len
    nut_far = S["z_idler_roof"] - I.nut.h
    assert nut_far - tip >= 2 * PARAMS.M5_PITCH and math.hypot(reach, iy) - I.slot_w / 2.0 >= D.bay_r + 5.0
    assert S["z_idler_roof"] == pytest.approx(S["z_front"] - I.roof) and I.roof >= 3.0
    _y0, y1, z0, z1 = motor_pocket(DEFAULT)
    half = I.nut.af / 2.0 + I.nut_fit
    assert iy - half - y1 >= 2.5
    assert y1 - I.entry < S["y_motor"] + D.motor.body_width / 2.0 < y1   # the way in starts under the motor's face
    assert iy + I.riser_dia / 2.0 < -D.ring_core_dia / 2.0 - 5.0      # the spacer off the pulley's core, which turns over the face
    y_face, y_back, z_low, z_top = roll_idler_setback()
    assert (y_face, y_back, z_top) == pytest.approx((y1, iy - half, z1)) and z_low < (z0 + z1) / 2.0
    assert z_low - (y_back - y_face) > back_point(0.0, D.back.opening_bottom, D.back.opening_depth)[2] + 5.0


def test_the_kit_is_parts_with_no_reference():
    """The axle and the nut bought, the spacer printed (an accent-coloured small print); none has a reference file."""
    for name, (pieces, volume, lo, size) in LOCKS.items():
        mod, shape = parts.load(name), parts.build(name)
        assert parts.GROUPS[name] == "joints" and name not in parts.unplaced() and len(shape.solids()) == pieces
        if name == SPACER:
            assert not parts.bought(name) and parts.accent(name)
            assert R.NO_REFERENCE[name] == "lib/forearm/roll.py:build_idler_spacer(DEFAULT)"
        else:
            assert parts.bought(name) and mod.PURCHASE_QTY == pieces
            assert R.NO_REFERENCE[name] == f"parts/joints/{name}.py:_envelope()"
        assert R.solid_volume(shape) == pytest.approx(volume, abs=1e-3)
        bb = shape.bounding_box()
        assert tuple(bb.min) == pytest.approx(lo, abs=1e-6) and tuple(bb.size) == pytest.approx(size, abs=1e-6)
    assert parts.load("roll_idler_screw").MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * shcs_volume(I.screw, I.screw_len))
    assert parts.load("roll_idler_nut").MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * nut_volume(I.nut))
    assert "gt2_idler_20t" not in parts.unplaced()


@pytest.fixture(scope="module")
def module():
    return built.model(M.forearm_roll_drive)


def _leaf(module, label):
    return next(c for c in module.children if c.label == label)


@pytest.mark.slow
def test_the_slot_and_channel_on_the_frame(module):
    block = _leaf(module, "forearm_roll_block")
    ix, iy = roll_idler_xy()
    out, inn = I.slot
    for x in (ix + out, ix, ix + inn):
        # nothing on the front face under the spacer (the frame prints on it): the face just behind it, whole beside the
        # slot, and nothing in front of it
        for dy in (I.slot_w / 2.0 + 0.5, I.riser_dia / 2.0, -I.riser_dia / 2.0):
            assert is_inside(block, x, iy + dy, S["z_front"] - 0.3) and not is_inside(block, x, iy + dy, S["z_front"] + 0.3), (x, dy)
        # the M5's slot through the plate, the roof and on past the screw's tip
        for z in (S["z_front"] - 1.0, S["z_idler_top"] - I.screw_len - I.tip_clear + 0.5):
            assert not is_inside(block, x, iy, z), (x, z)
    for x in (ix + out - I.slot_w / 2.0 - 0.5, ix + inn + I.slot_w / 2.0 + 0.5):                # the slot ends
        assert is_inside(block, x, iy, S["z_front"] - 0.3), x
    # the nut's channel: past the slot's ends both ways, closed beyond them (the side wall whole out to the front face's
    # facet), the roof over it; the nut's way in from the motor's pocket along +Y into it
    z_nut = S["z_idler_roof"] - I.nut.h / 2.0
    corner = I.nut.af / math.sqrt(3.0) + I.nut_fit
    for x in (ix + out, ix + inn):
        assert not is_inside(block, x, iy + I.nut.af / 2.0, z_nut) and is_inside(block, x, iy + I.nut.af / 2.0 + I.nut_fit + 0.5, z_nut)
    # the frame's outside at the nut's height: the facet's x there, or - the nut under the facet's foot - the side face
    # inside its outline's chamfer (EDGE_CHAMFER)
    x_out = min(D.block_x[1] - D.front_facet + (S["z_front"] - z_nut), D.block_x[1] - EDGE_CHAMFER)
    for x in (ix + out - corner - 0.5, ix + inn + corner + 0.5, x_out - 0.5):
        assert is_inside(block, x, iy, z_nut), x
    assert is_inside(block, ix + inn + corner - 1.0, iy, S["z_idler_roof"] + I.roof / 2.0)
    _y0, y1, _z0, _z1 = motor_pocket(DEFAULT)
    for y in (y1 - I.entry + 0.5, (y1 + iy) / 2.0):
        assert not is_inside(block, ix, y, z_nut), y
    assert is_inside(block, ix, y1 - I.entry - 0.5, S["z_motor_face"] + 0.5)
    # under it the pocket's far face set back to the channel's near side (the channel's floor whole behind it), from the
    # motor's face down to z_low, then the 45 deg slope back to the face, and the face below it
    y_face, y_back, z_low, z_top = roll_idler_setback()
    back, z_floor = y_back - y_face, S["z_idler_roof"] - I.nut.h - I.nut_fit
    assert z_floor < z_top
    for x, z, y_wall in ((ix, z_floor - 0.5, y_back), (10.0, z_floor - 0.5, y_back), (10.0, z_low + 0.5, y_back),
                         (10.0, z_low - back / 2.0, y_face + back / 2.0), (10.0, z_low - back - 1.0, y_face)):
        assert not is_inside(block, x, y_wall - 0.4, z) and is_inside(block, x, y_wall + 0.4, z), (x, z)


@pytest.mark.slow
def test_the_kit_fits(module):
    """The spacer on the frame's front face, the idler on its land, the belt on the idler's seat, the screw through
    the idler, the spacer, the slot and the nut, the nut on the channel's roof: touching, and nothing in the module
    overlapping the kit; the spacer clear of the belt and the pulley."""
    block, belt = _leaf(module, "forearm_roll_block"), _leaf(module, "forearm_roll_belt:forearm_roll")
    spacer, idler, screw, nut = (_leaf(module, label) for label in
                                 (SPACER, "gt2_idler_20t:forearm_roll", "roll_idler_screw", "roll_idler_nut"))
    for part in (spacer, idler, screw, nut):
        for other in module.children:
            if other is not part:
                assert interference(part, other) < 0.5, (part.label, other.label)
    assert closest_points(spacer, block).distance < 0.01 and closest_points(spacer, idler).distance < 0.01
    assert closest_points(belt, idler).distance < 0.01 and closest_points(idler, block).distance > 1.0
    assert closest_points(spacer, belt).distance >= 2.0 and closest_points(spacer, _leaf(module, "forearm_roll_pulley")).distance >= 5.0
    assert closest_points(nut, block).distance < 0.01 and closest_points(screw, block).distance >= (I.slot_w - I.screw.d) / 2.0 - 0.01
    assert closest_points(screw, spacer).distance >= (I.slot_w - I.screw.d) / 2.0 - 0.01
