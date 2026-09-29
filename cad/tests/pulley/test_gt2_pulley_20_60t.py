"""gt2_pulley_20_60t, a measured conversion (lib/pulley/, lib/reference.py MEASURED): no reference file is committed, so
the numbers measured 2026-09-29 on the SolidWorks export (sha256 55733855bdc5...) live HERE - each band's groove
arcs (the tangency solve lands on the export's), the build's surfaces, volume and bounding box, and feature probes
that name what moved."""
import math

import pytest

from lib import belts
from lib.pulley import COMPOUND
from lib.pulley.teeth import groove_arcs, groove_centres
from tests import built
from tests.helpers import is_inside
from tests.pulley.helpers import at, surfaces

# The centres of the export's groove arcs (the +y half of a groove turned onto +X), read off its cylinders: per band.
MEASURED_CENTRES = {
    60: {"bottom": (18.6495931710, 0.0), "blend": (18.7102218827, -0.0574628165),
         "flank": (18.8404602403, -0.3946513827), "tip": (18.6797798875, 0.7440679963)},
    20: {"bottom": (5.9171977237, 0.0), "blend": (5.9745253813, -0.0209524309),
         "flank": (6.1001366977, -0.3837880705), "tip": (5.9146900800, 0.7511610702)},
}

# The export: its volume, its bounding box (min, max) and its surfaces (tests/pulley/helpers.py surfaces()).
VOLUME = 10693.2600579
BBOX = ((-20.0445932, -1.2, -20.0445932), (20.0445932, 17.6, 20.0445932))
SURFACES = {
    ("cone", 45.0, -0.3, 0.0), ("cone", 45.0, 7.0, 7.3), ("cone", 45.0, 9.1, 9.4), ("cone", 45.0, 16.4, 16.7),
    ("cylinder", 0.15, 18.69459), ("cylinder", 0.555, 18.64959), ("cylinder", 0.63853, 18.71031),
    ("cylinder", 1.0, 18.84459),                                                    # the 60T's groove arcs
    ("cylinder", 0.15, 5.9622), ("cylinder", 0.555, 5.9172), ("cylinder", 0.61604, 5.97456),
    ("cylinder", 1.0, 6.1122),                                                      # the 20T's
    ("cylinder", 4.0, 0.0), ("cylinder", 6.1122, 0.0), ("cylinder", 7.3122, 0.0),   # the bore, the 20T's land, flanges
    ("cylinder", 18.84459, 0.0), ("cylinder", 20.04459, 0.0),                       # the 60T's land, flanges
    ("plane", -1, -1.2), ("plane", 1, 0.0), ("plane", -1, 7.0), ("plane", 1, 8.2),
    ("plane", 1, 9.4), ("plane", -1, 16.4), ("plane", 1, 17.6),
}


def test_params():
    c = COMPOUND
    assert c.teeth == belts.GT2_PULLEY_20_60T_TEETH == (60, 20)
    assert c.band_w - 1.0 == pytest.approx(belts.GT2_BELT_W)                              # each band: the belt + 1
    assert (c.flange_dia(0), c.flange_dia(1)) == pytest.approx((40.0892, 14.6244), abs=1e-4)
    assert (c.end_y, c.band_y0(1), c.top_y) == pytest.approx((-1.2, 9.4, 17.6))
    assert c.bore_dia == 8.0
    assert c.blend_r[0] == belts.GT2_BLEND_R and belts.GT2_GROOVE_R < c.blend_r[1] < belts.GT2_BLEND_R


def test_flank_offsets():
    """The export's flank centres: on the land, 1.2 deg (60T) and 3.6 deg (20T) across the centreline."""
    assert belts.flank_offset(60) == pytest.approx(0.3946513827, abs=1e-9)
    assert belts.flank_offset(20) == pytest.approx(0.3837880705, abs=1e-9)


@pytest.mark.parametrize("band", [0, 1])
def test_groove_centres(band):
    teeth = COMPOUND.teeth[band]
    for name, centre in groove_centres(teeth, COMPOUND.blend_r[band]).items():
        assert math.dist(centre, MEASURED_CENTRES[teeth][name]) < 1e-8, (teeth, name)
    arcs = groove_arcs(teeth, COMPOUND.blend_r[band])
    assert arcs[3][1] == pytest.approx((belts.pulley_od(teeth) / 2.0 - belts.GT2_TOOTH_DEPTH, 0.0))   # a tooth deep


@pytest.fixture(scope="module")
def pulley():
    return built.part("gt2_pulley_20_60t")


@pytest.mark.slow
def test_matches_the_export(pulley):
    assert pulley.is_valid and len(pulley.solids()) == 1
    assert pulley.volume == pytest.approx(VOLUME, rel=1e-6)
    bb = pulley.bounding_box()
    assert (*bb.min, *bb.max) == pytest.approx((*BBOX[0], *BBOX[1]), abs=1e-4)
    assert surfaces(pulley) == SURFACES


@pytest.mark.slow
def test_features(pulley):
    p = pulley
    for y in (-1.0, 8.2, 17.5):                                                          # the bore, end to end
        assert not is_inside(p, *at(3.9, y)) and is_inside(p, *at(4.1, y))
    # the 60T band: a land on +X, a groove half a pitch round
    assert is_inside(p, *at(18.7, 3.5, 0.0)) and not is_inside(p, *at(18.95, 3.5, 0.0))
    assert not is_inside(p, *at(18.3, 3.5, 3.0)) and is_inside(p, *at(17.9, 3.5, 3.0))
    assert is_inside(p, *at(20.0, -0.6)) and not is_inside(p, *at(20.1, -0.6))          # its lower flange ...
    assert is_inside(p, *at(19.8, -0.1)) and not is_inside(p, *at(19.9, -0.1))          # ... chamfered on the teeth's side
    assert is_inside(p, *at(20.0, 7.8)) and not is_inside(p, *at(20.1, 7.8))            # its upper flange ...
    assert is_inside(p, *at(19.8, 7.1)) and not is_inside(p, *at(19.9, 7.1))            # ... likewise
    assert is_inside(p, *at(7.4, 8.1)) and not is_inside(p, *at(7.4, 8.3))              # the shoulder at 8.2
    # the 20T band: a land on +X, a groove half a pitch round
    assert is_inside(p, *at(6.0, 12.9, 0.0)) and not is_inside(p, *at(6.2, 12.9, 0.0))
    assert not is_inside(p, *at(5.5, 12.9, 9.0)) and is_inside(p, *at(5.25, 12.9, 9.0))
    assert is_inside(p, *at(7.25, 8.6)) and not is_inside(p, *at(7.4, 8.6))             # its lower flange ...
    assert is_inside(p, *at(7.05, 9.3)) and not is_inside(p, *at(7.18, 9.3))            # ... chamfered on the teeth's side
    assert is_inside(p, *at(7.25, 17.2)) and not is_inside(p, *at(7.4, 17.2))           # its upper flange ...
    assert is_inside(p, *at(7.05, 16.5)) and not is_inside(p, *at(7.18, 16.5))          # ... likewise
    assert is_inside(p, *at(5.0, 17.5)) and not is_inside(p, *at(5.0, 17.7))            # the top
    assert is_inside(p, *at(10.0, -1.1)) and not is_inside(p, *at(10.0, -1.3))          # the bottom
