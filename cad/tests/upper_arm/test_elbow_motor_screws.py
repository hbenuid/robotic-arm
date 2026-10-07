"""elbow_motor_screws, the elbow motor's 4x M3: a purchased part with no reference (lib/reference.py NO_REFERENCE), which
keeps no reference file - THIS file locks its numbers (the pieces, the modelled volume and the bounding box, measured on
the build and written in) and what it is: every screw on one of the web's holes (j1_link's pad_holes, about the pad's
axis) with its head's bearing face on z=0, as long as the web and the thread into the motor.
The pattern in place - the heads under the web, the thread in the motor - is tests/test_mounts.py."""
import pytest

import parts
from lib import params as PARAMS
from lib import reference as R
from lib.upper_arm import DEFAULT as UPPER_ARM
from lib.upper_arm import pad_holes
from tests import built
from tests.helpers import is_inside

NAME = "elbow_motor_screws"
# (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCK = (4, 539.697, (-18.25, -18.25, -3.0), (36.5, 36.5, 13.0))


def _inside(shape, x: float, y: float, z: float) -> bool:
    return any(is_inside(s, x, y, z) for s in shape.solids())


def test_it_is_a_purchased_part_with_no_reference_in_the_joints_group():
    mod = parts.load(NAME)
    assert parts.GROUPS[NAME] == "joints" and parts.bought(NAME) and mod.PURCHASE_QTY == len(pad_holes(UPPER_ARM)) == LOCK[0]
    assert R.NO_REFERENCE[NAME] == f"parts/joints/{NAME}.py:_envelope()"


def test_the_order_line_and_mass_follow_the_web():
    mod = parts.load(NAME)
    assert mod.LENGTH == PARAMS.J1_MOTOR_SCREW_LEN == UPPER_ARM.arm.motor_plate_t + PARAMS.J1_MOTOR_SCREW_THREAD == 10.0
    assert PARAMS.J1_MOTOR_SCREW_THREAD < PARAMS.MOTOR_40.bolt_hole_depth                    # short of the holes' bottoms
    assert mod.PURCHASE_SPEC == "M3 x 10 socket head cap screw (ISO 4762)"
    steel = PARAMS.STEEL_DENSITY * LOCK[0]
    assert mod.MASS_G == PARAMS.ELBOW_MOTOR_SCREWS_MASS_G == pytest.approx(steel * PARAMS.shcs_volume(PARAMS.M3_SHCS, 10.0))
    # the pattern is the web's holes, j1_link's (x, z) about the pad's axis as the part's (x, -y)
    assert sorted(mod.POINTS) == sorted((x, -z) for x, z, _ in pad_holes(UPPER_ARM))


@pytest.mark.slow
def test_the_build_matches_its_numbers():
    solids, volume, lo, size = LOCK
    shape = built.part(NAME)
    assert len(shape.solids()) == solids
    assert R.solid_volume(shape) == pytest.approx(volume, abs=0.01)
    assert R.bbox_min(shape) == pytest.approx(lo, abs=1e-3) and R.bbox_size(shape) == pytest.approx(size, abs=1e-3)
    assert R.solid_volume(shape) == pytest.approx(solids * PARAMS.shcs_volume(PARAMS.M3_SHCS, 10.0), rel=1e-6)


@pytest.mark.slow
def test_each_screw_stands_on_a_hole_of_the_web():
    """Head in -Z under the bearing face z=0 (its socket open at the top), the shank in +Z to the length."""
    screws, size, length = built.part(NAME), PARAMS.M3_SHCS, PARAMS.J1_MOTOR_SCREW_LEN
    for x, y in parts.load(NAME).POINTS:
        assert _inside(screws, x, y, length - 0.1) and not _inside(screws, x, y, length + 0.1)
        assert _inside(screws, x + size.d / 2.0 - 0.1, y, length / 2.0) and not _inside(screws, x + size.d / 2.0 + 0.1, y, length / 2.0)
        assert _inside(screws, x + size.head_dia / 2.0 - 0.1, y, -size.head_h + 0.1)            # the head's rim, to its top
        assert not _inside(screws, x, y, -size.head_h + size.socket_depth - 0.1)                # the socket
        assert _inside(screws, x, y, -size.head_h + size.socket_depth + 0.1)
