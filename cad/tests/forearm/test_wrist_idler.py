"""The wrist belt's idler kit (lib/forearm/params.py IdlerParams) is NOT in the arm: DEFAULT's wrist motor sits at the
stock belt's centre distance, so its loop is the plain pair and its axle and its nut (wrist_idler_screw /
wrist_idler_nut) stay modelled, UNPLACED - the idler itself (gt2_idler_20t) is the roll belt's tensioner
(tests/forearm/test_roll_idler.py). The axle and the nut are purchased parts with no
reference (lib/reference.py NO_REFERENCE), so THIS file locks their numbers (pieces, the modelled volume and the bounding
box, measured on the build and written in). The option stays whole: with an idler (a ForearmConfig.idler, the motor
where the belt is slack) the loop goes round it and j2_link carries its rib, slot and nut channel.
Every belt's own loop: tests/test_belts.py."""
from dataclasses import replace

import pytest

import parts
from lib import params as PARAMS
from lib import reference as R
from lib.belts import GT2_IDLER_WIDTH, STANDARD_2GT_LENGTHS, belt_length, idler_pitch_r
from lib.fasteners import nut_volume, shcs_volume
from lib.forearm import DEFAULT, IdlerParams, wrist_belt_loop, wrist_idler_xy
from lib.forearm.link import build_link
from lib.geom import hex_circumdiameter
from tests.helpers import is_inside

I, W, R_END = IdlerParams(), DEFAULT.web, DEFAULT.roll_end
KIT = ("gt2_idler_20t", "wrist_idler_screw", "wrist_idler_nut")
WITH_IDLER = replace(DEFAULT, idler=I, motor_x=DEFAULT.slide_range[0])   # the motor at its slots' boss end: the belt slack
# name -> (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCKS = {
    "wrist_idler_screw": (1, 641.783, (-4.25, -4.25, -5.0), (8.5, 8.5, 25.0)),
    "wrist_idler_nut": (1, 168.216, (-4.0, -4.618802, 0.0), (8.0, 9.237604, 4.7)),
}


def test_the_arm_has_no_wrist_idler():
    """DEFAULT: the stock belt's loop is the 20T and the 90T alone, the stock length at the motor's centre distance (the
    exact path within 0.2 of the textbook formula's); the wrist kit's axle and nut are modelled, UNPLACED - off the arm
    and the buy list; the idler is placed, on the roll belt."""
    assert DEFAULT.idler is None
    loop = wrist_belt_loop()
    assert len(loop) == 2 and R_END.wrist_belt in STANDARD_2GT_LENGTHS
    assert belt_length(loop) == pytest.approx(R_END.wrist_belt, abs=0.2)
    assert set(KIT[1:]) <= set(parts.unplaced()) and KIT[0] not in parts.unplaced()


def test_an_idler_takes_up_a_slack_belt():
    """With an idler and the motor at its slots' boss end, the stock belt is longer than the open loop: the idler on
    the run leaving the 20T, IdlerParams.along from the motor, is pushed into the loop until the belt fits."""
    loop, open_loop = wrist_belt_loop(WITH_IDLER), wrist_belt_loop(WITH_IDLER, idler=False)
    assert len(loop) == 3 and belt_length(loop) == pytest.approx(R_END.wrist_belt, abs=1e-6)
    assert R_END.wrist_belt - belt_length(open_loop) > 5.0
    x, y, r = loop[1]
    assert (x, r) == (I.along, -idler_pitch_r()) and abs(y) > 0.5
    assert wrist_idler_xy(WITH_IDLER) == (WITH_IDLER.motor_x - x, -y)


def test_the_hardware_are_purchased_parts_with_no_reference():
    for name, (pieces, volume, lo, size) in LOCKS.items():
        mod, shape = parts.load(name), parts.build(name)
        assert parts.GROUPS[name] == "joints" and parts.bought(name) and mod.PURCHASE_QTY == pieces == len(shape.solids())
        assert R.NO_REFERENCE[name] == f"parts/joints/{name}.py:_envelope()"
        assert R.solid_volume(shape) == pytest.approx(volume, abs=1e-3)
        bb = shape.bounding_box()
        assert tuple(bb.min) == pytest.approx(lo, abs=1e-6) and tuple(bb.size) == pytest.approx(size, abs=1e-6)
    assert parts.load("wrist_idler_screw").MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * shcs_volume(I.screw, I.screw_len))
    assert parts.load("wrist_idler_nut").MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * nut_volume(I.nut))


@pytest.mark.slow
def test_the_web_carries_an_idler():
    """j2_link built with an idler: the rib under the web (rib_w across x), the M5 slot through rib and web (+/- travel
    along y), the nut's channel (its flats + nut_fit across x) from nut_floor up through the top face, 3 of web under it."""
    link = build_link(WITH_IDLER)
    xi, yi = wrist_idler_xy(WITH_IDLER)
    z_rib = R_END.wrist_belt_z + GT2_IDLER_WIDTH / 2.0      # the idler's top face = the rib's underside
    mid = (z_rib + W.z0) / 2.0
    assert not is_inside(link, xi, yi, mid) and not is_inside(link, xi, yi + I.travel, mid)            # the slot
    assert is_inside(link, xi + I.slot_w / 2.0 + 0.3, yi, mid)                                      # the rib beside it ...
    assert not is_inside(link, xi + I.rib_w / 2.0 + 0.3, yi, mid)                                   # ... rib_w wide
    assert not is_inside(link, xi, yi, z_rib - 0.2)                                                 # the idler's room under it
    half = (I.nut.af + I.nut_fit) / 2.0
    assert not is_inside(link, xi + half - 0.2, yi, I.nut_floor + 1.0) and is_inside(link, xi + half + 0.3, yi, I.nut_floor + 1.0)
    reach = I.travel + (hex_circumdiameter(I.nut.af) + I.nut_fit) / 2.0
    assert not is_inside(link, xi, yi + reach - 0.2, I.nut_floor + 1.0) and is_inside(link, xi, yi + reach + 0.3, I.nut_floor + 1.0)
    assert is_inside(link, xi + I.slot_w / 2.0 + 0.3, yi, I.nut_floor - 1.5) and I.nut_floor - W.z0 == pytest.approx(3.0)
