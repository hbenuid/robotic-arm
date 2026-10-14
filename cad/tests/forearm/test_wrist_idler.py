"""The wrist belt's idler (lib/forearm/params.py IdlerParams): the stock belt round it, the web's rib, slot and nut channel
that hold it, and its axle + nut - purchased parts with no reference (lib/reference.py NO_REFERENCE), so THIS file locks
their numbers (pieces, the modelled volume and the bounding box, measured on the build and written in).
Every belt's own loop: tests/test_belts.py; the mounts' interference budget: tests/test_mounts.py."""
import math

import pytest
from build123d import Location

import parts
from lib import params as PARAMS
from lib import placements as P
from lib import reference as R
from lib.belts import (
    GT2_IDLER_FLANGE_DIA,
    GT2_IDLER_WIDTH,
    GT2_PLD,
    GT2_TOOTH_DEPTH,
    STANDARD_2GT_LENGTHS,
    belt_length,
    belt_runs,
    belt_wraps,
    idler_pitch_r,
)
from lib.fasteners import nut_volume, shcs_volume
from lib.forearm import DEFAULT, wrist_belt_loop, wrist_idler_xy
from lib.geom import hex_circumdiameter
from lib.pulley.params import DEFAULT as PULLEY_90T
from tests import built
from tests.helpers import is_inside

I, W, R_END = DEFAULT.idler, DEFAULT.web, DEFAULT.roll_end
XI, YI = wrist_idler_xy()
Z_RIB = R_END.wrist_belt_z + GT2_IDLER_WIDTH / 2.0      # the idler's top face = the rib's underside
T20_FLANGE_R = 8.0                                      # [REFERENCE] the vendor 20T's flanges (Ø16)
# name -> (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCKS = {
    "wrist_idler_screw": (1, 641.783, (-4.25, -4.25, -5.0), (8.5, 8.5, 25.0)),
    "wrist_idler_nut": (1, 168.216, (-4.0, -4.618802, 0.0), (8.0, 9.237604, 4.7)),
}


def _dist_to_run(point, run) -> float:
    """A point's distance to a straight run (a segment)."""
    (x0, y0), (x1, y1), _ = run
    dx, dy = x1 - x0, y1 - y0
    t = max(0.0, min(1.0, ((point[0] - x0) * dx + (point[1] - y0) * dy) / (dx * dx + dy * dy)))
    return math.hypot(point[0] - (x0 + t * dx), point[1] - (y0 + t * dy))


def test_the_stock_belt_goes_round_the_idler():
    """The motor at its slide's wall end leaves the stock belt 11.4 longer than the open loop: the idler on the run
    leaving the 20T, pushed 18.9 into the loop, takes it up; the 20T then wraps more than half its teeth (the open loop:
    146 deg)."""
    loop, open_loop = wrist_belt_loop(), wrist_belt_loop(idler=False)
    assert R_END.wrist_belt in STANDARD_2GT_LENGTHS and len(loop) == 3 and len(open_loop) == 2
    assert belt_length(loop) == pytest.approx(R_END.wrist_belt, abs=1e-6)
    assert R_END.wrist_belt - belt_length(open_loop) == pytest.approx(11.419, abs=0.01)
    x, y, r = loop[1]
    assert (x, r) == (I.along, -idler_pitch_r()) and idler_pitch_r() == pytest.approx(6.426)
    push = idler_pitch_r() + _dist_to_run((x, y), belt_runs(open_loop)[0])   # the pitch line moved in at the idler
    assert push == pytest.approx(18.905, abs=0.01)
    wrap20 = math.degrees(belt_wraps(loop)[0][1])
    assert wrap20 == pytest.approx(192.8, abs=0.1) and math.degrees(belt_wraps(open_loop)[0][1]) == pytest.approx(146.0, abs=0.1)
    assert (XI, YI) == (DEFAULT.motor_x - x, -y) == PARAMS.J2_IDLER_XY


def test_the_idler_clears_the_pulleys_and_the_other_run():
    """Its flanges (Ø18) 7 from the 20T's, 13 from the 90T's, 4 from the other run's tooth tips."""
    loop = wrist_belt_loop()
    (x20, y20, _), (x, y, _), (x90, y90, _) = loop
    flange = GT2_IDLER_FLANGE_DIA / 2.0
    assert math.hypot(x - x20, y - y20) - flange - T20_FLANGE_R == pytest.approx(7.019, abs=0.01)
    assert math.hypot(x - x90, y - y90) - flange - PULLEY_90T.flange_dia / 2.0 > 13.0
    tips = belt_runs(loop, -GT2_PLD - GT2_TOOTH_DEPTH)[2]   # the run back to the 20T: its tooth tips
    assert _dist_to_run((x, y), tips) - flange == pytest.approx(4.32, abs=0.02)


def test_the_slot_tensions_the_belt():
    """Over the slot's +/- travel the loop runs 3.2 shorter to 3.7 longer than the stock belt: room to fit it and to
    tension it."""
    a, (x, y, r), b = wrist_belt_loop()
    out = belt_length([a, (x, y - I.travel, r), b]) - R_END.wrist_belt
    into = belt_length([a, (x, y + I.travel, r), b]) - R_END.wrist_belt
    assert out == pytest.approx(-3.234, abs=0.01) and into == pytest.approx(3.698, abs=0.01)


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
    assert "UNPLACED" not in vars(parts.load("gt2_idler_20t"))


@pytest.mark.slow
def test_the_web_carries_the_idler():
    """j2_link: the rib under the web (rib_w across x), the M5 slot through rib and web (+/- travel along y), the nut's
    channel (its flats + nut_fit across x) from nut_floor up through the top face, 3 of web under it."""
    link = built.part("j2_link")
    mid = (Z_RIB + W.z0) / 2.0
    assert not is_inside(link, XI, YI, mid) and not is_inside(link, XI, YI + I.travel, mid)            # the slot
    assert is_inside(link, XI + I.slot_w / 2.0 + 0.3, YI, mid)                                      # the rib beside it ...
    assert not is_inside(link, XI + I.rib_w / 2.0 + 0.3, YI, mid)                                   # ... rib_w wide
    assert not is_inside(link, XI, YI, Z_RIB - 0.2)                                                 # the idler's room under it
    half = (I.nut.af + I.nut_fit) / 2.0
    assert not is_inside(link, XI + half - 0.2, YI, I.nut_floor + 1.0) and is_inside(link, XI + half + 0.3, YI, I.nut_floor + 1.0)
    reach = I.travel + (hex_circumdiameter(I.nut.af) + I.nut_fit) / 2.0
    assert not is_inside(link, XI, YI + reach - 0.2, I.nut_floor + 1.0) and is_inside(link, XI, YI + reach + 0.3, I.nut_floor + 1.0)
    assert is_inside(link, XI + I.slot_w / 2.0 + 0.3, YI, I.nut_floor - 1.5) and I.nut_floor - W.z0 == pytest.approx(3.0)


@pytest.mark.slow
def test_the_idler_axle_and_nut_in_place():
    """In j2_link's frame: the idler level with the wrist 90T's band, its top on the rib; the axle's head under it, its tip
    through the nut and under the web's top face (the wrist motor's face); the nut on its channel's floor."""
    j2 = P.location("j2_link#1", "world")

    def local(key: str, at=(0.0, 0.0, 0.0)):
        return (j2.inverse() * P.location(key, "world") * Location(at)).position

    idler = local("gt2_idler_20t#1")
    assert (idler.X, idler.Y) == pytest.approx((XI, YI), abs=1e-5)
    assert idler.Z + GT2_IDLER_WIDTH / 2.0 == pytest.approx(R_END.wrist_belt_z, abs=1e-6) == PARAMS.J2_BELT_Z
    band90 = local("gt2_pulley_90t#4", (0.0, PULLEY_90T.band_y1 / 2.0, 0.0))
    assert band90.Z == pytest.approx(R_END.wrist_belt_z, abs=1e-5)                                  # the 90T's band
    assert idler.Z + GT2_IDLER_WIDTH == pytest.approx(Z_RIB)                                        # on the rib
    head, tip = local("wrist_idler_screw#1"), local("wrist_idler_screw#1", (0.0, 0.0, I.screw_len))
    nut = local("wrist_idler_nut#1")
    assert head.Z == pytest.approx(idler.Z) and nut.Z == pytest.approx(I.nut_floor)
    assert nut.Z + I.nut.h <= tip.Z < W.z1 == PARAMS.J2_MOTOR_WEB_FACE_Z
