"""The GT2 belts (parts/base/yaw_belt, parts/joints/{elbow,forearm_roll,wrist}_belt): purchased parts with no reference
(lib/reference.py NO_REFERENCE) - THIS file locks their numbers (the modelled band's volume and bounding box, measured
on the build and written in) and what they are: each a stock length, its loop that long along its pitch line
(lib/belts.py), the band on its pulleys' tooth tips (and an idler's seat, were one in its loop); in the arm, each loop's
circles on its pulleys' axes in their tooth bands' plane. The wrist idler kit (unplaced): tests/forearm/test_wrist_idler.py."""
import math

import pytest
from build123d import Location

import parts
from assemblies import forearm_roll_drive as ROLL
from lib import placements as P
from lib import reference as R
from lib.belts import (
    GT2_BELT_BACK,
    GT2_BELT_MASS_G_PER_M,
    GT2_BELT_W,
    GT2_IDLER_WIDTH,
    GT2_PLD,
    STANDARD_2GT_LENGTHS,
    belt_length,
    belt_wraps,
)
from lib.datum import to_location
from lib.forearm import DEFAULT as FOREARM
from lib.pulley.params import DEFAULT as PULLEY_90T
from lib.pulley.params import YAW as PULLEY_120T
from tests import built
from tests.helpers import is_inside

T20_HUB = FOREARM.drive.t20_hub     # the vendor 20T's tooth band from its hub face (along its +X)
# name -> (group, volume mm^3, bbox min, bbox size): the build's own numbers
LOCKS = {
    "yaw_belt": ("base", 1136.33, (-6.742198, -38.573186, -3.0), (118.381942, 77.146373, 6.0)),
    "elbow_belt": ("joints", 1059.849, (-6.742198, -29.02389, -3.0), (113.766087, 58.04778, 6.0)),
    "forearm_roll_belt": ("joints", 1059.849, (-6.742198, -29.02389, -3.0), (109.766087, 58.04778, 6.0)),
    "wrist_belt": ("joints", 1059.993, (-6.742198, -29.02389, -3.0), (117.737762, 58.04778, 6.0)),
}
# a mounted belt -> its loop's circles' pulleys (key, its band centre in its own frame: the 20T's along +X, a 90T's /
# 120T's along +Y)
PULLEYS = {
    "yaw_belt#1": [("gt2_pulley_20t#3", (T20_HUB, 0.0, 0.0)), ("gt2_pulley_120t#1", (0.0, PULLEY_120T.band_y1 / 2.0, 0.0))],
    "elbow_belt#1": [("gt2_pulley_20t#2", (T20_HUB, 0.0, 0.0)), ("gt2_pulley_90t#3", (0.0, PULLEY_90T.band_y1 / 2.0, 0.0))],
    "wrist_belt#1": [("gt2_pulley_20t#4", (T20_HUB, 0.0, 0.0)), ("gt2_pulley_90t#4", (0.0, PULLEY_90T.band_y1 / 2.0, 0.0))],
}


def test_every_belt_is_a_purchased_stock_length_with_no_reference():
    assert {n for n in R.NO_REFERENCE if n.endswith("_belt")} == set(LOCKS)
    for name, (group, *_) in LOCKS.items():
        mod = parts.load(name)
        assert parts.bought(name) and parts.GROUPS[name] == group and mod.PURCHASE_QTY == 1
        assert R.NO_REFERENCE[name] == f"parts/{group}/{name}.py:_envelope()"
        assert mod.LENGTH in STANDARD_2GT_LENGTHS and mod.PURCHASE_SPEC.startswith(f"{mod.LENGTH}-2GT closed GT2 belt")
        assert mod.MASS_G == pytest.approx(mod.LENGTH * GT2_BELT_MASS_G_PER_M / 1000.0)


def test_each_loop_is_its_belt_long():
    """Along the pitch line, exactly: a two-pulley loop at the textbook centre distance (lib/belts.py
    closed_belt_length()) runs a little long - 0.04 round a 90T, 0.23 round the 120T (the wrist's has no idler: a plain
    pair too)."""
    for name in LOCKS:
        mod = parts.load(name)
        assert abs(belt_length(mod.loop()) - mod.LENGTH) <= 0.25, name
    assert belt_length(parts.load("wrist_belt").loop()) - parts.load("wrist_belt").LENGTH == pytest.approx(0.038, abs=0.002)
    assert belt_length(parts.load("yaw_belt").loop()) - parts.load("yaw_belt").LENGTH == pytest.approx(0.233, abs=0.002)


@pytest.mark.parametrize("name", sorted(LOCKS))
def test_the_band(name):
    """One solid, GT2_BELT_W wide about z=0; at the middle of each wrap its land on the pulley's tooth tips (GT2_PLD
    inside the pitch line) and its back GT2_BELT_BACK outside it - round an idler the back on its seat."""
    _, volume, lo, size = LOCKS[name]
    mod, band = parts.load(name), parts.build(name)
    assert len(band.solids()) == 1 and band.is_valid
    assert R.solid_volume(band) == pytest.approx(volume, abs=1e-3)
    bb = band.bounding_box()
    assert tuple(bb.min) == pytest.approx(lo, abs=1e-6) and tuple(bb.size) == pytest.approx(size, abs=1e-6)
    assert (bb.min.Z, bb.max.Z) == pytest.approx((-GT2_BELT_W / 2.0, GT2_BELT_W / 2.0))
    loop = mod.loop()
    for (x, y, r), (a0, sweep) in zip(loop, belt_wraps(loop), strict=True):
        a = a0 + sweep / 2.0

        def at(offset: float, x=x, y=y, r=r, a=a) -> tuple[float, float, float]:
            """The point `offset` outside the pitch line at the wrap's middle (a signed radius, as lib/belts.py)."""
            return (x + (r + offset) * math.cos(a), y + (r + offset) * math.sin(a), 0.0)

        assert is_inside(band, *at(-GT2_PLD + 0.05)) and not is_inside(band, *at(-GT2_PLD - 0.05)), (name, r)
        assert is_inside(band, *at(GT2_BELT_BACK - 0.05)) and not is_inside(band, *at(GT2_BELT_BACK + 0.05)), (name, r)


@pytest.mark.slow
@pytest.mark.parametrize("key", sorted(PULLEYS))
def test_each_mounted_belt_runs_on_its_pulleys(key):
    """In the arm: the loop's pulley circles on their pulleys' axes, in their tooth bands' plane (0.01)."""
    belt = P.location(key, "world")
    circles = [c for c in parts.load(key.split("#")[0]).loop() if c[2] > 0]
    for (x, y, _), (pulley, band) in zip(circles, PULLEYS[key], strict=True):
        want = (P.location(pulley, "world") * Location(band)).position
        got = (belt * Location((x, y, 0.0))).position
        assert (got - want).length < 0.01, (key, pulley, (got - want).length)


def test_the_roll_belt_runs_on_its_pulleys():
    """In the roll module: the belt's row on the motor's axis at the ring's mid-plane, its 20T's band there, the ring
    (on the roll axis) on the loop's second circle, its tensioner's idler (the third, on the belt's back) on the idler's
    row, its channel centred on the belt (tests/forearm/test_roll_idler.py: the idler itself)."""
    rows = {part: frame for part, _, frame in ROLL.OCCURRENCES}
    belt, t20 = to_location(rows["forearm_roll_belt"]), to_location(rows["gt2_pulley_20t"])
    (x0, y0, _), (x1, y1, _), (x2, y2, r2) = parts.load("forearm_roll_belt").loop()
    assert ((belt * Location((x0, y0, 0.0))).position - (t20 * Location((T20_HUB, 0.0, 0.0))).position).length < 1e-6
    ring = (belt * Location((x1, y1, 0.0))).position
    assert (ring.X, ring.Y, ring.Z) == pytest.approx((0.0, 0.0, ROLL.S["z_ring_mid"]), abs=1e-6)
    idler = (Location(rows["gt2_idler_20t"]) * Location((0.0, 0.0, GT2_IDLER_WIDTH / 2.0))).position   # a bare position row
    assert r2 < 0.0 and ((belt * Location((x2, y2, 0.0))).position - idler).length < 1e-6


@pytest.mark.slow
def test_the_belts_ride_line_to_line():
    """In the arm (built), each belt touches its pulleys and overlaps them by no more than float noise."""
    from cadgen.geometry import closest_points

    for key, pulleys in PULLEYS.items():
        belt = built.placed(key)
        for pulley, _ in pulleys:
            assert closest_points(belt, built.placed(pulley)).distance < 0.01, (key, pulley)
