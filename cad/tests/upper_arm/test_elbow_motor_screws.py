"""elbow_motor_screws, the elbow motor's 4x M3 SHCS - the 4 that ship with its gearbox, through the gearbox's input plate
into the motor: a purchased part with no reference (lib/reference.py NO_REFERENCE), which keeps no reference file - THIS
file locks its numbers (the pieces, the modelled volume and the bounding box, measured on the build and written in) and
what it is: every screw on the NEMA 17 square with its head's bearing face on z=0, as long under the head as the plate
and the thread into the motor. The pattern in place - the heads in the gearbox's corner channels, the thread in the
motor - is tests/test_mounts.py."""
import pytest

import parts
from lib import params as PARAMS
from lib import reference as R
from tests import built
from tests.helpers import is_inside

NAME = "elbow_motor_screws"
SCREW = PARAMS.M3_SHCS
G = PARAMS.EG17_G5
# (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCK = (4, 483.148, (-18.25, -18.25, -3.0), (36.5, 36.5, 11.0))


def _inside(shape, x: float, y: float, z: float) -> bool:
    return any(is_inside(s, x, y, z) for s in shape.solids())


def test_it_is_a_purchased_part_with_no_reference_in_the_joints_group():
    mod = parts.load(NAME)
    assert parts.GROUPS[NAME] == "joints" and parts.bought(NAME) and mod.PURCHASE_QTY == LOCK[0]
    assert R.NO_REFERENCE[NAME] == f"parts/joints/{NAME}.py:_envelope()"


def test_an_m3_x_8_through_the_gearboxs_plate_into_the_motor():
    """The gearbox's input plate (EG17_G5.plate_t) under the heads, the rest in the motor's tapped holes - short of their
    bottoms, the stock length under the drive's rule (J1_MOTOR_SCREW_THREAD) through that plate; the pattern the NEMA 17
    square, the plate's holes, their heads clear of the corner channels' walls."""
    mod = parts.load(NAME)
    assert mod.PURCHASE_SPEC == "M3 x 8 socket head cap screw (ISO 4762)" and mod.SCREW == SCREW
    assert mod.LENGTH == PARAMS.J1_MOTOR_SCREW_LEN == 8.0 < G.plate_t + PARAMS.J1_MOTOR_SCREW_THREAD
    thread = mod.LENGTH - G.plate_t                                                    # 3.3
    assert 3.0 < thread < PARAMS.MOTOR_40.bolt_hole_depth
    half = G.plate_sp / 2.0
    assert half == PARAMS.NEMA17_BOLT_SP / 2.0 and G.plate_hole > SCREW.d
    assert sorted(mod.POINTS) == sorted((sx * half, sy * half) for sx in (1, -1) for sy in (1, -1))
    assert half - SCREW.head_dia / 2.0 > G.channel                                     # the heads in the channels
    steel = PARAMS.STEEL_DENSITY * LOCK[0]
    assert mod.MASS_G == PARAMS.ELBOW_MOTOR_SCREWS_MASS_G == pytest.approx(steel * PARAMS.shcs_volume(SCREW, 8.0))


@pytest.mark.slow
def test_the_build_matches_its_numbers():
    solids, volume, lo, size = LOCK
    shape = built.part(NAME)
    assert len(shape.solids()) == solids
    assert R.solid_volume(shape) == pytest.approx(volume, abs=0.01)
    assert R.bbox_min(shape) == pytest.approx(lo, abs=1e-3) and R.bbox_size(shape) == pytest.approx(size, abs=1e-3)
    assert R.solid_volume(shape) == pytest.approx(solids * PARAMS.shcs_volume(SCREW, 8.0), rel=1e-6)


@pytest.mark.slow
def test_each_screw_stands_on_the_square():
    """Head in -Z under the bearing face z=0 (its socket open at the top), the shank in +Z to the length."""
    screws, length = built.part(NAME), PARAMS.J1_MOTOR_SCREW_LEN
    for x, y in parts.load(NAME).POINTS:
        assert _inside(screws, x, y, length - 0.1) and not _inside(screws, x, y, length + 0.1)
        assert _inside(screws, x + SCREW.d / 2.0 - 0.1, y, length / 2.0) and not _inside(screws, x + SCREW.d / 2.0 + 0.1, y, length / 2.0)
        assert _inside(screws, x + SCREW.head_dia / 2.0 - 0.1, y, -SCREW.head_h + 0.1)          # the head's rim, to its top
        assert not _inside(screws, x, y, -SCREW.head_h + SCREW.socket_depth - 0.1)              # the socket
        assert _inside(screws, x, y, -SCREW.head_h + SCREW.socket_depth + 0.1)
