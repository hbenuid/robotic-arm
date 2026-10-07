"""gt2_pulley_90t, parametric (lib/pulley/): the groove's tangency solve lands on the SolidWorks arcs, the LEGACY
configuration reproduces the SolidWorks part feature by feature (the volume / bbox match is
tests/test_reference_match.py's; here the features are probed by name so a regression names what moved, and its
surfaces are the reference's own), and DEFAULT - what the part builds - opens the bolt holes, counterbores them for the
heads and changes nothing else."""
import itertools
import math
from dataclasses import replace

import pytest

from lib import belts
from lib.fasteners import M4_CLEAR, M4_SHCS
from lib.pulley import DEFAULT, LEGACY
from lib.pulley.teeth import groove_arcs, groove_centres
from tests import built
from tests.helpers import is_inside
from tests.pulley.helpers import at, surfaces

# The centres of the SolidWorks groove's arcs (the +y half of the groove on +X), read off the reference's cylinders.
MEASURED_CENTRES = {"bottom": (28.1988897565, 0.0), "blend": (28.2555855858, -0.0613465476),
                    "flank": (28.3911220340, -0.3964406093), "tip": (28.2341206498, 0.7427918364)}


def test_params():
    assert LEGACY.teeth == belts.GT2_PULLEY_90T_TEETH
    assert (LEGACY.end_y, LEGACY.face_y) == belts.GT2_PULLEY_90T_FACE_Y
    assert LEGACY.bolt_points() == belts.pulley_90t_bolt_points()
    assert LEGACY.band_y1 - 1.0 == pytest.approx(belts.GT2_BELT_W)                     # the band: the belt + 1
    assert LEGACY.hub_dia == 30.0 and LEGACY.flange_dia == pytest.approx(59.188, abs=1e-3)   # the 6806's bore; the bbox
    assert DEFAULT == replace(LEGACY, hole_dia=M4_CLEAR, counterbore=(M4_SHCS.head_dia + 0.4, belts.GT2_PULLEY_90T_HEAD_SEAT))
    assert LEGACY.counterbore == (0.0, 0.0)


def test_groove_centres():
    for name, centre in groove_centres(LEGACY.teeth).items():
        assert math.dist(centre, MEASURED_CENTRES[name]) < 1e-8, name


def test_groove_arcs():
    arcs = groove_arcs(LEGACY.teeth)
    r_land = belts.pulley_od(LEGACY.teeth) / 2.0
    assert len(arcs) == 7
    for (_, _, end), (start, _, _) in itertools.pairwise(arcs):
        assert end == start                                                             # one chain
    for (start, mid, end), (m_start, m_mid, m_end) in zip(arcs, reversed(arcs), strict=True):   # symmetric
        assert (*start, *mid, *end) == pytest.approx((m_end[0], -m_end[1], m_mid[0], -m_mid[1], m_start[0], -m_start[1]))
    assert math.hypot(*arcs[0][0]) == pytest.approx(r_land) and arcs[0][0][1] < 0.0 < arcs[-1][2][1]   # land to land
    assert arcs[3][1] == pytest.approx((r_land - belts.GT2_TOOTH_DEPTH, 0.0))              # the bottom, a tooth deep
    # the groove's width at the land, as the reference's fillets meet it: 1.507 deg either side of the centreline
    assert math.degrees(math.atan2(arcs[-1][2][1], arcs[-1][2][0])) == pytest.approx(1.5070071840, abs=1e-8)


@pytest.fixture(scope="module")
def legacy():
    return built.legacy("gt2_pulley_90t")


@pytest.fixture(scope="module")
def pulley():
    return built.part("gt2_pulley_90t")


@pytest.mark.slow
def test_legacy_surfaces_are_the_references(legacy):
    from lib import reference
    ref = reference.load("gt2_pulley_90t")
    assert legacy.is_valid and len(legacy.solids()) == 1
    assert surfaces(legacy) == surfaces(ref)
    assert legacy.volume == pytest.approx(ref.volume, rel=1e-7)


@pytest.mark.slow
def test_legacy_features(legacy):
    leg, c = legacy, LEGACY                 # at()'s default 45 deg keeps a probe clear of the bolt holes (on the axes)
    assert is_inside(leg, *at(14.8, -10.0)) and not is_inside(leg, *at(15.2, -10.0))       # the journal
    assert is_inside(leg, *at(17.2, -5.5)) and not is_inside(leg, *at(17.2, -6.4))         # the ring ...
    assert not is_inside(leg, *at(17.2, -4.5)) and not is_inside(leg, *at(17.5, -5.5))     # ... Ø34.76, 1.5 high
    assert is_inside(leg, *at(14.8, 0.5)) and not is_inside(leg, *at(15.2, 0.5))           # the hub up to the step
    assert is_inside(leg, *at(17.3, 1.3)) and not is_inside(leg, *at(17.7, 1.3))           # the step
    assert not is_inside(leg, *at(22.0, 1.6)) and is_inside(leg, *at(22.0, 2.0))           # open under the web
    assert is_inside(leg, *at(26.7, -1.0)) and not is_inside(leg, *at(26.3, -1.0))         # the rim's inside
    assert is_inside(leg, *at(29.5, -0.6)) and not is_inside(leg, *at(29.7, -0.6))         # the lower flange ...
    assert is_inside(leg, *at(29.5, -0.25)) and not is_inside(leg, *at(29.5, -0.1))        # ... its chamfer
    assert is_inside(leg, *at(29.5, 7.8)) and not is_inside(leg, *at(29.5, 7.1))           # the upper flange, chamfered
    assert not is_inside(leg, *at(27.9, 3.5, 0.0)) and is_inside(leg, *at(27.5, 3.5, 0.0))  # a groove on +X ...
    assert is_inside(leg, *at(28.3, 3.5, 2.0)) and not is_inside(leg, *at(28.5, 3.5, 2.0))  # ... a land between two
    assert not is_inside(leg, *at(6.0, 0.0)) and is_inside(leg, *at(6.5, 0.0))              # the bore
    for x, z in c.bolt_points():
        u = (x / c.bolt_r, z / c.bolt_r)
        for y in (c.end_y + 0.5, 0.0, c.face_y - 0.5):                                     # the bolt holes, end to face
            assert not is_inside(leg, x, y, z) and not is_inside(leg, x + 1.85 * u[0], y, z + 1.85 * u[1])
            assert is_inside(leg, x + 2.05 * u[0], y, z + 2.05 * u[1])


@pytest.mark.slow
def test_default_opens_and_counterbores_the_bolt_holes(pulley, legacy):
    (cb_dia, cb_depth), r_hole = DEFAULT.counterbore, DEFAULT.hole_dia / 2.0
    holes = 4 * math.pi * (r_hole ** 2 - (LEGACY.hole_dia / 2.0) ** 2) * (DEFAULT.face_y - DEFAULT.end_y)
    counterbores = 4 * math.pi * ((cb_dia / 2.0) ** 2 - r_hole ** 2) * cb_depth
    assert legacy.volume - pulley.volume == pytest.approx(holes + counterbores, abs=1e-3)
    for x, z in DEFAULT.bolt_points():
        u = (x / DEFAULT.bolt_r, z / DEFAULT.bolt_r)
        for y, r in ((DEFAULT.end_y + 0.5, r_hole), (0.0, r_hole), (DEFAULT.face_y - cb_depth - 0.2, r_hole),
                     (DEFAULT.face_y - cb_depth + 0.2, cb_dia / 2.0), (DEFAULT.face_y - 0.2, cb_dia / 2.0)):
            assert not is_inside(pulley, x + (r - 0.1) * u[0], y, z + (r - 0.1) * u[1]), (x, z, y)
            assert is_inside(pulley, x + (r + 0.1) * u[0], y, z + (r + 0.1) * u[1]), (x, z, y)
    lb, pb = legacy.bounding_box(), pulley.bounding_box()
    assert (pb.min.X, pb.min.Y, pb.min.Z, pb.max.X, pb.max.Y, pb.max.Z) == pytest.approx(
        (lb.min.X, lb.min.Y, lb.min.Z, lb.max.X, lb.max.Y, lb.max.Z), abs=1e-6)
