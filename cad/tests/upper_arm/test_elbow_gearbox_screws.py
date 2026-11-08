"""elbow_gearbox_screws, the elbow gearbox's 4x M4 SHCS, up through j1_link's web into the gearbox's output face: a
purchased part with no reference (lib/reference.py NO_REFERENCE), which keeps no reference file - THIS file locks its
numbers (the pieces, the modelled volume and the bounding box, measured on the build and written in) and what it is:
every screw on one of the web's holes (j1_link's pad_holes, about the pad's axis) with its head's bearing face on z=0,
as long under the head as the web under the heads and the thread into the gearbox. The pattern in place - the heads in
the web's pockets, the thread in the gearbox - is tests/test_mounts.py."""
import pytest

import parts
from lib import params as PARAMS
from lib import reference as R
from lib.upper_arm import DEFAULT as UPPER_ARM
from lib.upper_arm import pad_holes
from tests import built
from tests.helpers import is_inside

NAME = "elbow_gearbox_screws"
SCREW = UPPER_ARM.arm.seat_screw
G = PARAMS.EG17_G5
# (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCK = (4, 1156.584, (-21.0, -21.0, -4.0), (42.0, 42.0, 16.0))


def _inside(shape, x: float, y: float, z: float) -> bool:
    return any(is_inside(s, x, y, z) for s in shape.solids())


def test_it_is_a_purchased_part_with_no_reference_in_the_joints_group():
    mod = parts.load(NAME)
    assert parts.GROUPS[NAME] == "joints" and parts.bought(NAME) and mod.PURCHASE_QTY == len(pad_holes(UPPER_ARM)) == LOCK[0]
    assert R.NO_REFERENCE[NAME] == f"parts/joints/{NAME}.py:_envelope()"


def test_an_m4_x_12_through_the_web_into_the_gearbox():
    """The web under the heads (ArmParams.web_t: the web web_t + head_h thick, flush with the slab's underside, as it was
    under the motor's M3s), J1_GEARBOX_SCREW_THREAD in the gearbox's 8 deep holes; the pattern the gearbox's bolt
    circle, on its axes."""
    mod, a = parts.load(NAME), UPPER_ARM.arm
    assert mod.PURCHASE_SPEC == "M4 x 12 socket head cap screw (ISO 4762)"
    assert mod.SCREW == SCREW == PARAMS.M4_SHCS and a.seat_pocket_dia > SCREW.head_dia
    assert mod.LENGTH == PARAMS.J1_GEARBOX_SCREW_LEN == a.web_t + PARAMS.J1_GEARBOX_SCREW_THREAD == 12.0
    assert a.web_t + SCREW.head_h == 11.0 and PARAMS.J1_GEARBOX_SCREW_THREAD < G.bolt_depth
    r = G.bolt_pcd / 2.0
    assert sorted(mod.POINTS) == sorted(((r, 0.0), (-r, 0.0), (0.0, r), (0.0, -r)))
    assert sorted(mod.POINTS) == sorted((x, -z) for x, z, _ in pad_holes(UPPER_ARM))
    steel = PARAMS.STEEL_DENSITY * LOCK[0]
    assert mod.MASS_G == PARAMS.ELBOW_GEARBOX_SCREWS_MASS_G == pytest.approx(steel * PARAMS.shcs_volume(SCREW, 12.0))


@pytest.mark.slow
def test_the_build_matches_its_numbers():
    solids, volume, lo, size = LOCK
    shape = built.part(NAME)
    assert len(shape.solids()) == solids
    assert R.solid_volume(shape) == pytest.approx(volume, abs=0.01)
    assert R.bbox_min(shape) == pytest.approx(lo, abs=1e-3) and R.bbox_size(shape) == pytest.approx(size, abs=1e-3)
    assert R.solid_volume(shape) == pytest.approx(solids * PARAMS.shcs_volume(SCREW, 12.0), rel=1e-6)


@pytest.mark.slow
def test_each_screw_stands_on_a_hole_of_the_web():
    """Head in -Z under the bearing face z=0 (its socket open at the top), the shank in +Z to the length."""
    screws, length = built.part(NAME), PARAMS.J1_GEARBOX_SCREW_LEN
    for x, y in parts.load(NAME).POINTS:
        assert _inside(screws, x, y, length - 0.1) and not _inside(screws, x, y, length + 0.1)
        assert _inside(screws, x + SCREW.d / 2.0 - 0.1, y, length / 2.0) and not _inside(screws, x + SCREW.d / 2.0 + 0.1, y, length / 2.0)
        assert _inside(screws, x + SCREW.head_dia / 2.0 - 0.1, y, -SCREW.head_h + 0.1)          # the head's rim, to its top
        assert not _inside(screws, x, y, -SCREW.head_h + SCREW.socket_depth - 0.1)              # the socket
        assert _inside(screws, x, y, -SCREW.head_h + SCREW.socket_depth + 0.1)
