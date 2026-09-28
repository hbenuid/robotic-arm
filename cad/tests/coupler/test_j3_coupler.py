"""j3_coupler, parametric (lib/coupler/): the LEGACY configuration reproduces the SolidWorks part feature by feature
(the volume / bbox match is tests/test_reference_match.py's; here the features are probed by name so a regression
names what moved), its stub is the one the forearm roll drive's block repeats at the elbow, and DEFAULT - what the
part builds - adds exactly two things: the shoulder on the upper wrist bearing's inner ring, and PULLEY_SEAT_SHIFT
more stub, on through the lip to the re-seated wrist 90T it bolts to."""
import math
from dataclasses import replace

import pytest

from lib.bearings import PULLEY_SEAT_SHIFT
from lib.coupler import DEFAULT, LEGACY, flange_bolt_points, pulley_bolt_points
from lib.forearm import DEFAULT as FOREARM
from tests import built
from tests.helpers import is_inside


def test_layout():
    assert len(pulley_bolt_points(LEGACY)) == len(flange_bolt_points(LEGACY)) == 4
    assert all(math.isclose(math.hypot(x, z), LEGACY.pulley_bolt_r) for x, z in pulley_bolt_points(LEGACY))
    d = FOREARM.drive
    # the roll drive's block repeats the coupler's features at the elbow, the inner-ring shoulder included
    assert (d.stub_dia, d.journal_dia, d.boss_dia, d.pulley_bolt_r) == (LEGACY.stub_dia, LEGACY.journal_dia, LEGACY.lip_dia[1],
                                                                        LEGACY.pulley_bolt_r)
    assert LEGACY.step is None and DEFAULT.step[0] == d.step_dia
    assert d.step_x[1] - d.step_x[0] == pytest.approx(DEFAULT.step[1] - DEFAULT.journal_y1)
    assert d.stub_x[1] - d.stub_x[0] == pytest.approx(DEFAULT.stub_y1 - DEFAULT.step[1])


@pytest.fixture(scope="module")
def legacy():
    return built.legacy("j3_coupler")


@pytest.fixture(scope="module")
def coupler():
    return built.part("j3_coupler")


@pytest.mark.slow
def test_legacy_features(legacy):
    leg = legacy
    assert leg.is_valid and len(leg.solids()) == 1
    assert is_inside(leg, 38.5, 5, 0) and not is_inside(leg, 39.5, 5, 0)          # the flange
    assert is_inside(leg, 30, 12, 0) and not is_inside(leg, 28.5, 12, 0)           # the dust-lip ring ...
    assert not is_inside(leg, 31.5, 12, 0) and not is_inside(leg, 30, 14.5, 0)     # ... Ø58 / 62, 4 high
    assert is_inside(leg, 19.5, 15, 0) and not is_inside(leg, 19.5, 15.6, 5)       # the journal
    assert is_inside(leg, 14.5, 21.5, 2) and not is_inside(leg, 15.5, 18, 0)       # the stub
    assert not is_inside(leg, 0, 10, 6) and is_inside(leg, 0, 10, 6.5)             # the bore
    for x, z in pulley_bolt_points(LEGACY):
        assert not is_inside(leg, x, 18, z) and not is_inside(leg, x, 1, z)        # the 90T's bolts, their nut pockets:
        assert not is_inside(leg, x + 3.2, 1, z) and is_inside(leg, x + 3.6, 1, z)   # flats at +/- 3.425 in x ...
        assert not is_inside(leg, x, 1, z + 3.8) and is_inside(leg, x, 1, z + 4.1)   # ... a corner along z
        assert is_inside(leg, x + 3.2, 3.2, z)                                       # ... 2.8 deep
    for x, z in flange_bolt_points(LEGACY):
        side = (x + 3.0 * z / LEGACY.flange_bolt_r, z + 3.0 * x / LEGACY.flange_bolt_r)   # 3 off the bolt's axis
        assert not is_inside(leg, x, 3, z) and not is_inside(leg, side[0], 8, side[1])     # the bolt, its counterbore ...
        assert is_inside(leg, side[0], 5, side[1])                                         # ... which stops at y 6


@pytest.mark.slow
def test_default_adds_the_shoulder_and_the_longer_stub(coupler, legacy):
    assert replace(DEFAULT, step=None, stub_y1=LEGACY.stub_y1) == LEGACY
    assert DEFAULT.stub_y1 == LEGACY.stub_y1 + PULLEY_SEAT_SHIFT
    dia, y1 = DEFAULT.step
    r = dia / 2.0
    assert is_inside(coupler, r - 0.2, (DEFAULT.journal_y1 + y1) / 2.0, 0) and not is_inside(legacy, r - 0.2, (DEFAULT.journal_y1 + y1) / 2.0, 0)
    assert not is_inside(coupler, r - 0.2, y1 + 0.1, 0)                           # the stub above it
    ring = math.pi * (r ** 2 - (DEFAULT.stub_dia / 2.0) ** 2) * (y1 - DEFAULT.journal_y1)
    # the stub's extension: its annulus round the bore, less the 90T's 4 bolt holes through it
    extension = math.pi * ((DEFAULT.stub_dia / 2.0) ** 2 - (DEFAULT.bore_dia / 2.0) ** 2 - 4 * (DEFAULT.pulley_bolt_dia / 2.0) ** 2) * PULLEY_SEAT_SHIFT
    assert coupler.volume - legacy.volume == pytest.approx(ring + extension, abs=1e-3)
    y = LEGACY.stub_y1 + PULLEY_SEAT_SHIFT / 2.0
    assert is_inside(coupler, 14.5, y, 2) and not is_inside(legacy, 14.5, y, 2)
    assert coupler.bounding_box().max.Y == pytest.approx(DEFAULT.stub_y1, abs=1e-6)
    assert (coupler.bounding_box().max.X, coupler.bounding_box().max.Z) == pytest.approx((legacy.bounding_box().max.X, legacy.bounding_box().max.Z))
