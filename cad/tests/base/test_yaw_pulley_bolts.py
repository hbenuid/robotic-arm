"""yaw_pulley_screws / yaw_pulley_nuts, the base_yaw 120T's 4x M4 and their nuts: purchased parts with no reference
(lib/reference.py NO_REFERENCE), which keep no reference file - THIS file locks their numbers (the pieces, the modelled
volume and the bounding box, measured on the build and written in) and what they are: every screw on the pulley's hole
circle (the 90T's, which the 120T keeps) with its head's bearing face on z=0, every nut on it with a corner where
j1_coupler's pockets have theirs.
The patterns in place - the heads in the pulley's counterbores, the nuts in the pockets with the designed press, the reach past the
nuts - are tests/test_mounts.py."""
import math

import pytest

import parts
from lib import params as PARAMS
from lib import reference as R
from lib.geom import hex_circumdiameter
from lib.yaw_coupler import DEFAULT as YAW_COUPLER
from tests import built
from tests.helpers import is_inside

POINTS = PARAMS.pulley_90t_bolt_points()
# name -> (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCKS = {
    "yaw_pulley_screws": (4, 2564.018, (-14.5, -14.5, -4.0), (29.0, 29.0, 44.0)),
    "yaw_pulley_nuts": (4, 382.322, (-14.903743, -14.903743, 0.0), (29.807486, 29.807486, 3.2)),
}


def _inside(shape, x: float, y: float, z: float) -> bool:
    return any(is_inside(s, x, y, z) for s in shape.solids())


def test_they_are_purchased_parts_with_no_reference_in_the_base_group():
    assert set(LOCKS) == {n for n in R.NO_REFERENCE if parts.bought(n)}
    for name in LOCKS:
        mod = parts.load(name)
        assert parts.GROUPS[name] == "base" and parts.bought(name) and mod.PURCHASE_QTY == len(POINTS) == LOCKS[name][0]
        assert R.NO_REFERENCE[name] == f"parts/base/{name}.py:_envelope()"


def test_the_order_lines_and_masses_follow_the_hub():
    screws, nuts = (parts.load(n) for n in LOCKS)
    length = YAW_COUPLER.hub.pulley_screw_len
    assert screws.LENGTH == length == 40.0 and screws.PURCHASE_SPEC == "M4 x 40 socket head cap screw (ISO 4762)"
    assert nuts.PURCHASE_SPEC == "M4 hex nut (ISO 4032)"
    steel = PARAMS.STEEL_DENSITY * len(POINTS)
    assert screws.MASS_G == PARAMS.YAW_PULLEY_SCREWS_MASS_G == pytest.approx(steel * PARAMS.shcs_volume(PARAMS.M4_SHCS, length))
    assert nuts.MASS_G == PARAMS.PULLEY_NUTS_MASS_G == pytest.approx(steel * PARAMS.nut_volume(PARAMS.M4_NUT))
    assert nuts.CORNER_DEG == YAW_COUPLER.hub.hole_deg - 90.0 == -45.0


@pytest.mark.slow
@pytest.mark.parametrize("name", list(LOCKS))
def test_the_build_matches_its_numbers(name):
    solids, volume, lo, size = LOCKS[name]
    shape = built.part(name)
    assert len(shape.solids()) == solids
    assert R.solid_volume(shape) == pytest.approx(volume, abs=0.01)
    assert R.bbox_min(shape) == pytest.approx(lo, abs=1e-3) and R.bbox_size(shape) == pytest.approx(size, abs=1e-3)


@pytest.mark.slow
def test_the_modelled_volumes_are_the_analytic_ones():
    """lib/fasteners.py's volumes (what the masses are estimated from) are those of the modelled geometry."""
    n = len(POINTS)
    assert R.solid_volume(built.part("yaw_pulley_screws")) == pytest.approx(n * PARAMS.shcs_volume(PARAMS.M4_SHCS, 40.0), rel=1e-6)
    assert R.solid_volume(built.part("yaw_pulley_nuts")) == pytest.approx(n * PARAMS.nut_volume(PARAMS.M4_NUT), rel=1e-6)


@pytest.mark.slow
def test_each_screw_stands_on_the_hole_circle():
    """Head in -Z under the bearing face z=0 (its socket open at the top), the shank in +Z to the length."""
    screws, size, length = built.part("yaw_pulley_screws"), PARAMS.M4_SHCS, YAW_COUPLER.hub.pulley_screw_len
    for x, y in POINTS:
        assert _inside(screws, x, y, length - 0.1) and not _inside(screws, x, y, length + 0.1)
        assert _inside(screws, x + size.d / 2.0 - 0.1, y, length / 2.0) and not _inside(screws, x + size.d / 2.0 + 0.1, y, length / 2.0)
        assert _inside(screws, x + size.head_dia / 2.0 - 0.1, y, -size.head_h + 0.1)            # the head's rim, to its top
        assert not _inside(screws, x, y, -size.head_h + size.socket_depth - 0.1)                # the socket
        assert _inside(screws, x, y, -size.head_h + size.socket_depth + 0.1)


@pytest.mark.slow
def test_each_nut_has_a_corner_where_the_couplers_pockets_have_theirs():
    """On the hole circle, standing on z=0, bored at the thread's diameter, a corner at CORNER_DEG from the pattern's
    +X (a flat 30 degrees on): the coupler's Z once the pattern is turned with the pulley (tests/test_mounts.py checks
    the press in the pockets)."""
    nuts, size = built.part("yaw_pulley_nuts"), PARAMS.M4_NUT
    corner_r = hex_circumdiameter(size.af) / 2.0
    corner = math.radians(parts.load("yaw_pulley_nuts").CORNER_DEG)
    for x, y in POINTS:
        assert not _inside(nuts, x, y, size.h / 2.0)                                            # the bore
        assert not _inside(nuts, x + size.d / 2.0 - 0.1, y, size.h / 2.0) and _inside(nuts, x + size.d / 2.0 + 0.1, y, size.h / 2.0)
        for k in range(6):
            a = corner + k * math.pi / 3.0
            r = corner_r - 0.05
            assert _inside(nuts, x + r * math.cos(a), y + r * math.sin(a), size.h / 2.0), (x, y, k)
            flat = a + math.pi / 6.0
            assert not _inside(nuts, x + r * math.cos(flat), y + r * math.sin(flat), size.h / 2.0), (x, y, k)
        assert not _inside(nuts, x, y + size.af / 2.0 - 0.1, size.h + 0.1) and not _inside(nuts, x, y + size.af / 2.0 - 0.1, -0.1)
