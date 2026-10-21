"""wrist_motor_screws, the wrist-pitch motor's 4x M3 SHCS - the elbow motor's screws (elbow_motor_screws) in the elbow
motor's seat: a purchased part with no reference (lib/reference.py NO_REFERENCE), which keeps no reference file - THIS
file locks its numbers (the pieces, the modelled volume and the bounding box, measured on the build and written in) and
what it is: every screw on one of j2_link's motor holes (about the motor's axis) with its head's bearing face on z=0,
as long under the head as the web under the counterbores and the thread into the motor. In place: the heads flush in
the counterbores, the pattern clear of the motor, its 20T and the wrist belt's idler kit; the counterbores probed on
j2_link. The motor's frame on the screws - each on a bolt axis, the thread in the motor - is tests/test_mounts.py."""
import pytest

import parts
from lib import params as PARAMS
from lib import reference as R
from lib.forearm import DEFAULT, LEGACY, motor_hole_points
from lib.upper_arm import DEFAULT as UPPER_ARM
from tests import built
from tests.forearm.helpers import in_host
from tests.helpers import interference, is_inside

NAME = "wrist_motor_screws"
H, W = DEFAULT.holes, DEFAULT.web
SCREW = H.screw
# (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCK = (4, 596.246, (-18.25, -18.25, -3.0), (36.5, 36.5, 15.0))


def _inside(shape, x: float, y: float, z: float) -> bool:
    return any(is_inside(s, x, y, z) for s in shape.solids())


def test_it_is_a_purchased_part_with_no_reference_in_the_joints_group():
    mod = parts.load(NAME)
    assert parts.GROUPS[NAME] == "joints" and parts.bought(NAME) and mod.PURCHASE_QTY == len(motor_hole_points(DEFAULT)) == LOCK[0]
    assert R.NO_REFERENCE[NAME] == f"parts/joints/{NAME}.py:_envelope()" and NAME not in parts.unplaced()


def test_it_is_the_elbow_motors_screw_in_the_elbow_motors_seat():
    """The same screw, counterbore and thread as the elbow motor's (lib/upper_arm/params.py ArmParams): j2_link's web is as
    thick as the elbow's web under its motor, so the heads sit flush in counterbores head_h deep with the elbow's 8 of
    web under them, and the M3 x 12 reaches the drive's thread into the motor."""
    mod, a = parts.load(NAME), UPPER_ARM.arm
    assert mod.PURCHASE_SPEC == parts.load("elbow_motor_screws").PURCHASE_SPEC == "M3 x 12 socket head cap screw (ISO 4762)"
    assert mod.SCREW == SCREW == a.motor_screw == PARAMS.M3_SHCS and H.pocket_dia == a.motor_pocket_dia > SCREW.head_dia
    assert H.thread == PARAMS.J1_MOTOR_SCREW_THREAD == PARAMS.MOTOR_40.bolt_hole_depth - 0.5
    assert W.thickness == a.motor_plate_t + SCREW.head_h and W.thickness - SCREW.head_h == a.motor_plate_t == 8.0
    assert mod.LENGTH == PARAMS.J2_MOTOR_SCREW_LEN == PARAMS.J1_MOTOR_SCREW_LEN == 12.0
    steel = PARAMS.STEEL_DENSITY * LOCK[0]
    assert mod.MASS_G == PARAMS.WRIST_MOTOR_SCREWS_MASS_G == pytest.approx(steel * PARAMS.shcs_volume(SCREW, 12.0))
    # the pattern is the web's holes about the motor's axis - the 31 mm square; none on LEGACY's slide
    half = PARAMS.NEMA17_BOLT_SP / 2.0
    assert sorted(mod.POINTS) == sorted((sx * half, sy * half) for sx in (1, -1) for sy in (1, -1))
    assert sorted(mod.POINTS) == sorted((x - DEFAULT.motor_x, y) for x, y in motor_hole_points(DEFAULT))
    assert motor_hole_points(LEGACY) == []


def test_the_counterbores_clear_the_idler_nuts_channel():
    """The -x counterbores beside the wrist idler's nut channel (lib/forearm/params.py IdlerParams): 1.5 mm+ of web
    across, and below its floor (the channel runs down from the top face to nut_floor, the counterbores up to head_h)."""
    i = DEFAULT.idler
    xi = DEFAULT.motor_x - i.along
    assert DEFAULT.motor_x - H.bolt_half - H.pocket_dia / 2.0 - (xi + (i.nut.af + i.nut_fit) / 2.0) >= 1.5
    assert W.z0 + SCREW.head_h <= i.nut_floor


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
    screws, length = built.part(NAME), PARAMS.J2_MOTOR_SCREW_LEN
    for x, y in parts.load(NAME).POINTS:
        assert _inside(screws, x, y, length - 0.1) and not _inside(screws, x, y, length + 0.1)
        assert _inside(screws, x + SCREW.d / 2.0 - 0.1, y, length / 2.0) and not _inside(screws, x + SCREW.d / 2.0 + 0.1, y, length / 2.0)
        assert _inside(screws, x + SCREW.head_dia / 2.0 - 0.1, y, -SCREW.head_h + 0.1)          # the head's rim, to its top
        assert not _inside(screws, x, y, -SCREW.head_h + SCREW.socket_depth - 0.1)              # the socket
        assert _inside(screws, x, y, -SCREW.head_h + SCREW.socket_depth + 0.1)


@pytest.mark.slow
def test_the_counterbores_in_j2_link():
    """Each hole's counterbore: pocket_dia across, from the web's underside up to head_h, the M3 clearance hole on through
    the web above it; the web whole round the counterbore."""
    link = built.part("j2_link")
    r, top = H.pocket_dia / 2.0, W.z0 + SCREW.head_h
    for x, y in motor_hole_points(DEFAULT):
        assert not is_inside(link, x + r - 0.2, y, W.z0 + 0.2) and not is_inside(link, x + r - 0.2, y, top - 0.2)
        assert is_inside(link, x + r - 0.2, y, top + 0.2) and is_inside(link, x + r + 0.2, y, W.z0 + 0.2)
        assert not is_inside(link, x + H.bolt_dia / 2.0 - 0.2, y, top + 0.2) and not is_inside(link, x, y, W.z1 - 0.2)


@pytest.mark.slow
def test_in_place_the_heads_are_flush_and_clear_of_the_motor_and_the_idler_kit():
    """The placed pattern in j2_link's frame: the heads' ends on the web's underside, nothing of them below it; no
    interference with j2_link, the motor, its 20T or the wrist idler kit."""
    screws = in_host("wrist_motor_screws#1")
    bb = screws.bounding_box()
    assert bb.min.Z == pytest.approx(W.z0, abs=1e-6) and bb.max.Z == pytest.approx(W.z1 + H.thread, abs=1e-6)
    for key in ("j2_link#1", "nema17_40mm#3", "gt2_pulley_20t#4", "gt2_idler_20t#3", "wrist_idler_screw#1", "wrist_idler_nut#1"):
        assert interference(screws, in_host(key)) == pytest.approx(0.0, abs=1e-6), key
