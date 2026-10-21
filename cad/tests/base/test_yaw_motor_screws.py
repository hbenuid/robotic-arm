"""yaw_motor_screws, the base_yaw motor's 4x M3 SHCS - the roll motor's M3 x 8 in the wrist and roll motors' seat
(lib/forearm/params.py MotorHoleParams): a purchased part with no reference (lib/reference.py NO_REFERENCE), which keeps
no reference file - THIS file locks its numbers (the pieces, the modelled volume and the bounding box, measured on the
build and written in) and what it is: every screw on one of the motor mount's motor holes (about the motor's axis) with
its head's bearing face on z=0, as long under the head as the plate and its pad under the counterbores and the thread
into the motor. In place: the heads flush with the plate's top, clear of the mount, the motor's board, its 20T and the
belt's idler kit. The seat on the mount (the pad, the holes, the counterbores) is tests/base/test_motor_mount.py; the
motor's frame on the screws - each on a bolt axis, the thread in the motor - tests/test_mounts.py."""
import pytest

import parts
from lib import params as PARAMS
from lib import reference as R
from lib.base import DEFAULT, motor_face_y, motor_holes, motor_screw_y
from lib.cycloidal import DEFAULT_CONFIG
from lib.forearm import DEFAULT as FOREARM
from tests import built
from tests.helpers import interference, is_inside

NAME = "yaw_motor_screws"
M, PL = DEFAULT.motor, DEFAULT.plate
SCREW = M.holes.screw
# (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCK = (4, 483.148, (-18.25, -18.25, -3.0), (36.5, 36.5, 11.0))


def _inside(shape, x: float, y: float, z: float) -> bool:
    return any(is_inside(s, x, y, z) for s in shape.solids())


def test_it_is_a_purchased_part_with_no_reference_in_the_base_group():
    mod = parts.load(NAME)
    assert parts.GROUPS[NAME] == "base" and parts.bought(NAME) and mod.PURCHASE_QTY == len(motor_holes(DEFAULT)) == LOCK[0]
    assert R.NO_REFERENCE[NAME] == f"parts/base/{NAME}.py:_envelope()" and NAME not in parts.unplaced()


def test_it_is_the_roll_motors_screw_in_the_wrist_motors_seat():
    """The roll motor's screw (parts/joints/roll_motor_screws) in the wrist and roll motors' counterbores: the plate (5)
    and its pad (2) give the heads the roll frame's 7 mm plate, so 4 under the heads and the M3 x 8 reaches the drive's
    thread into the motor."""
    mod = parts.load(NAME)
    assert mod.PURCHASE_SPEC == parts.load("roll_motor_screws").PURCHASE_SPEC == "M3 x 8 socket head cap screw (ISO 4762)"
    assert mod.SCREW == SCREW == PARAMS.M3_SHCS and M.holes == FOREARM.holes and M.holes.pocket_dia > SCREW.head_dia
    assert PL.y[1] - PL.y[0] + M.pad_t == FOREARM.drive.pad_t == 7.0
    assert motor_screw_y() == pytest.approx(PL.y[1] - SCREW.head_h) == PARAMS.YAW_MOTOR_SCREW_Y
    assert mod.LENGTH == PARAMS.YAW_MOTOR_SCREW_LEN == PARAMS.ROLL_MOTOR_SCREW_LEN == 8.0
    assert mod.LENGTH == pytest.approx(motor_screw_y() - motor_face_y() + M.holes.thread)
    m48 = DEFAULT_CONFIG.motor                                   # the 48 mm motor's tapped holes: the drive's rule
    assert M.holes.thread == m48.bolt_hole_depth - m48.motor_bolt_thread_margin == 4.0
    steel = PARAMS.STEEL_DENSITY * LOCK[0]
    assert mod.MASS_G == PARAMS.YAW_MOTOR_SCREWS_MASS_G == pytest.approx(steel * PARAMS.shcs_volume(SCREW, 8.0))
    # the pattern is the plate's holes about the motor's axis - the 31 mm square
    half = PARAMS.NEMA17_BOLT_SP / 2.0
    assert sorted(mod.POINTS) == sorted((sx * half, sy * half) for sx in (1, -1) for sy in (1, -1))
    assert sorted(mod.POINTS) == sorted((x - M.centre[0], z - M.centre[1]) for x, z in motor_holes(DEFAULT))


@pytest.mark.slow
def test_the_build_matches_its_numbers():
    solids, volume, lo, size = LOCK
    shape = built.part(NAME)
    assert len(shape.solids()) == solids
    assert R.solid_volume(shape) == pytest.approx(volume, abs=0.01)
    assert R.bbox_min(shape) == pytest.approx(lo, abs=1e-3) and R.bbox_size(shape) == pytest.approx(size, abs=1e-3)
    assert R.solid_volume(shape) == pytest.approx(solids * PARAMS.shcs_volume(SCREW, 8.0), rel=1e-6)


@pytest.mark.slow
def test_each_screw_stands_on_a_hole_of_the_plate():
    """Head in -Z under the bearing face z=0 (its socket open at the top), the shank in +Z to the length."""
    screws, length = built.part(NAME), PARAMS.YAW_MOTOR_SCREW_LEN
    for x, y in parts.load(NAME).POINTS:
        assert _inside(screws, x, y, length - 0.1) and not _inside(screws, x, y, length + 0.1)
        assert _inside(screws, x + SCREW.d / 2.0 - 0.1, y, length / 2.0) and not _inside(screws, x + SCREW.d / 2.0 + 0.1, y, length / 2.0)
        assert _inside(screws, x + SCREW.head_dia / 2.0 - 0.1, y, -SCREW.head_h + 0.1)          # the head's rim, to its top
        assert not _inside(screws, x, y, -SCREW.head_h + SCREW.socket_depth - 0.1)              # the socket
        assert _inside(screws, x, y, -SCREW.head_h + SCREW.socket_depth + 0.1)


@pytest.mark.slow
def test_in_place_the_heads_are_flush_and_clear_of_the_mount_and_the_belts_kit():
    """The placed pattern in the base's frame: the heads' ends on the plate's top, nothing of them above it, the tips
    the thread under the motor's face; no interference with the mount, the motor's board, its 20T, the belt or the
    belt's idler kit (the thread in the motor's holes: tests/test_mounts.py)."""
    screws = built.placed(f"{NAME}#1")
    bb = screws.bounding_box()
    assert bb.max.Y == pytest.approx(PL.y[1], abs=1e-6) and bb.min.Y == pytest.approx(motor_face_y() - M.holes.thread, abs=1e-6)
    for key in ("base_motor_mount#1", "mks_servo42d#1", "gt2_pulley_20t#3", "yaw_belt#1", "gt2_idler_20t#2",
                "yaw_idler_spacer#1", "yaw_idler_screw#1", "yaw_idler_nut#1"):
        assert interference(screws, built.placed(key)) == pytest.approx(0.0, abs=1e-6), key
