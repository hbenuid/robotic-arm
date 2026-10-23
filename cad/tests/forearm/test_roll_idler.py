"""The roll belt's tensioner (lib/forearm/params.py RollIdlerParams; cad/AGENTS.md "Belts"): the stock roll belt is
longer than the open loop at the drive's design centre distance, so an idler (gt2_idler_20t) on its back, on the run
back from the ring, takes up the rest (lib/forearm/layout.py roll_belt_loop, roll_idler_xy). It stands on a riser on
the roll frame's front face - the riser's land under the idler's bearing inner ring - on an M5 (roll_idler_screw) down
through the riser's slot into a nut (roll_idler_nut) in a channel under the face, closed at both ends, the nut in from
the motor's pocket: the slot and the channel along module X, +/- travel. The axle and the nut are purchased parts with no reference (lib/reference.py
NO_REFERENCE), so THIS file locks their numbers (pieces, the modelled volume and the bounding box, measured on the
build and written in). The idler keeps clear of the belt's far run, the ring's flange and the 20T's hub at its place
and at its slot's deep end; the riser, the slot and the channel are probed on the frame; the kit fits in the module."""
import math

import pytest
from cadgen.geometry import closest_points

import parts
from assemblies import forearm_roll_drive as M
from lib import params as PARAMS
from lib import reference as R
from lib.belts import GT2_IDLER_FLANGE_DIA, GT2_IDLER_WIDTH, STANDARD_2GT_LENGTHS, belt_length, belt_runs, idler_pitch_r
from lib.fasteners import nut_volume, shcs_volume
from lib.forearm import DEFAULT, motor_pocket, roll_belt_loop, roll_idler_xy, stack_positions
from tests import built
from tests.helpers import interference, is_inside

D, I, S = DEFAULT.drive, DEFAULT.drive.idler, stack_positions(DEFAULT)
# name -> (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCKS = {
    "roll_idler_screw": (1, 739.958, (-4.25, -4.25, -5.0), (8.5, 8.5, 30.0)),
    "roll_idler_nut": (1, 168.216, (-4.618802, -4.0, 0.0), (9.237604, 8.0, 4.7)),
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


@pytest.mark.parametrize("push", [0.0, I.travel])
def test_the_idler_clears_the_far_run_the_ring_and_the_20t(push):
    """At its place and at its slot's deep end (travel further in): CLEAR off the belt's far run and the ring's flange,
    which turn in its plane; well off the 20T's hub."""
    got = _clearances(push)
    assert got["far run"] >= CLEAR and got["ring"] >= CLEAR and got["20T hub"] >= 10.0, got


def test_the_riser_the_screw_and_the_nut():
    """The idler level with the belt on the riser's land; the screw's head on the idler's top, the shank through the
    riser and the roof into the nut, its tip 2 pitches or more past the nut and inside its hole, its hole clear of the
    shaft's bay; the nut's bearing face on the channel's roof, the channel 2.5 or more off the motor's pocket, under the
    front face, the nut's way in from the pocket starting entry inside its top edge, over the motor's face; the riser 3 mm
    or more round the slot."""
    assert S["z_idler"] + GT2_IDLER_WIDTH / 2.0 == pytest.approx(S["z_ring_mid"])
    assert S["z_idler"] - I.land_h > S["z_front"] and I.land_w < I.riser_w and I.slot_w < I.land_w
    assert (I.riser_w - I.slot_w) / 2.0 >= 3.0
    tip = S["z_idler_top"] - I.screw_len
    nut_far = S["z_idler_roof"] - I.nut.h
    ix, iy = roll_idler_xy()
    assert nut_far - tip >= 2 * PARAMS.M5_PITCH and math.hypot(abs(ix) + I.travel, iy) - I.slot_w / 2.0 >= D.bay_r + 5.0
    assert S["z_idler_roof"] == pytest.approx(S["z_front"] - I.roof) and I.roof >= 3.0
    _y0, y1, _z0, _z1 = motor_pocket(DEFAULT)
    assert iy - (I.nut.af / 2.0 + I.nut_fit) - y1 >= 2.5
    assert y1 - I.entry < S["y_motor"] + D.motor.body_width / 2.0 < y1   # the way in starts under the motor's face, which shuts it
    assert iy + I.riser_w / 2.0 < -D.bearing_boss_dia / 2.0 - 5.0      # the riser off the bearing boss


def test_the_hardware_are_purchased_parts_with_no_reference():
    for name, (pieces, volume, lo, size) in LOCKS.items():
        mod, shape = parts.load(name), parts.build(name)
        assert parts.GROUPS[name] == "joints" and parts.bought(name) and mod.PURCHASE_QTY == pieces == len(shape.solids())
        assert R.NO_REFERENCE[name] == f"parts/joints/{name}.py:_envelope()" and name not in parts.unplaced()
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
def test_the_riser_slot_and_channel_on_the_frame(module):
    block = _leaf(module, "forearm_roll_block")
    ix, iy = roll_idler_xy()
    z_riser, z_land = (S["z_front"] + S["z_idler"] - I.land_h) / 2.0, S["z_idler"] - I.land_h / 2.0
    # the riser and its land, stadiums along X over the travel
    for x in (ix - I.travel, ix, ix + I.travel):
        assert is_inside(block, x, iy + I.riser_w / 2.0 - 0.5, z_riser) and not is_inside(block, x, iy + I.riser_w / 2.0 + 0.5, z_riser)
        assert is_inside(block, x, iy - I.land_w / 2.0 + 0.3, z_land) and not is_inside(block, x, iy - I.land_w / 2.0 - 0.3, z_land)
        # the M5's slot through the land, the riser, the roof and on past the screw's tip
        for z in (z_land, z_riser, S["z_front"] - 1.0, S["z_idler_top"] - I.screw_len - I.tip_clear + 0.5):
            assert not is_inside(block, x, iy, z), (x, z)
    assert not is_inside(block, ix, iy, S["z_idler"] + 0.5)
    # the nut's channel: past the slot's ends both ways, closed beyond them (the side face whole), the roof over it; the
    # nut's way in from the motor's pocket along +Y into it
    z_nut = S["z_idler_roof"] - I.nut.h / 2.0
    reach = I.travel + I.nut.af / math.sqrt(3.0) + I.nut_fit
    for x in (ix - I.travel, ix + I.travel):
        assert not is_inside(block, x, iy + I.nut.af / 2.0, z_nut) and is_inside(block, x, iy + I.nut.af / 2.0 + I.nut_fit + 0.5, z_nut)
    for x in (ix - reach - 0.5, ix + reach + 0.5, D.block_x[1] - 0.5):
        assert is_inside(block, x, iy, z_nut), x
    assert is_inside(block, ix + reach - 1.0, iy, S["z_idler_roof"] + I.roof / 2.0)
    _y0, y1, _z0, _z1 = motor_pocket(DEFAULT)
    for y in (y1 - I.entry + 0.5, (y1 + iy) / 2.0):
        assert not is_inside(block, ix, y, z_nut), y
    assert is_inside(block, ix, y1 - I.entry - 0.5, S["z_motor_face"] + 0.5)


@pytest.mark.slow
def test_the_kit_fits(module):
    """The idler on the land, the belt on the idler's seat, the screw through the idler, the slot and the nut, the nut
    on the channel's roof: touching, and nothing in the module overlapping the kit."""
    block, belt = _leaf(module, "forearm_roll_block"), _leaf(module, "forearm_roll_belt:forearm_roll")
    idler, screw, nut = (_leaf(module, label) for label in ("gt2_idler_20t:forearm_roll", "roll_idler_screw", "roll_idler_nut"))
    for part in (idler, screw, nut):
        for other in module.children:
            if other is not part:
                assert interference(part, other) < 0.5, (part.label, other.label)
    assert closest_points(idler, block).distance < 0.01 and closest_points(belt, idler).distance < 0.01
    assert closest_points(nut, block).distance < 0.01 and closest_points(screw, block).distance >= (I.slot_w - I.screw.d) / 2.0 - 0.01
