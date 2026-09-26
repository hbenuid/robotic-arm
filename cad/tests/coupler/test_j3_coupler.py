"""j3_coupler, parametric (lib/coupler/): the LEGACY configuration reproduces the SolidWorks part feature by feature
(the volume / bbox match is tests/test_reference_match.py's; here the features are probed by name so a regression
names what moved), and its stub is the one the forearm roll drive's block repeats at the elbow."""
import math

import pytest

import parts
from lib.coupler import DEFAULT, LEGACY, flange_bolt_points, pulley_bolt_points
from lib.forearm import DEFAULT as FOREARM
from tests.helpers import is_inside


def test_layout():
    assert len(pulley_bolt_points(LEGACY)) == len(flange_bolt_points(LEGACY)) == 4
    assert all(math.isclose(math.hypot(x, z), LEGACY.pulley_bolt_r) for x, z in pulley_bolt_points(LEGACY))
    d = FOREARM.drive
    # the roll drive's block repeats the coupler's features at the elbow (its stub ends 22 - 15.3 from the journal)
    assert (d.stub_dia, d.journal_dia, d.boss_dia, d.pulley_bolt_r) == (LEGACY.stub_dia, LEGACY.journal_dia, LEGACY.lip_dia[1],
                                                                        LEGACY.pulley_bolt_r)
    assert d.stub_x[1] - d.stub_x[0] == pytest.approx(LEGACY.stub_y1 - LEGACY.journal_y1)


@pytest.fixture(scope="module")
def legacy():
    from lib.coupler.body import build_coupler
    return build_coupler(LEGACY)


@pytest.fixture(scope="module")
def coupler():
    return parts.build("j3_coupler")


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
def test_default_is_legacy(coupler, legacy):
    assert DEFAULT == LEGACY
    assert coupler.volume == pytest.approx(legacy.volume, abs=1e-6)
