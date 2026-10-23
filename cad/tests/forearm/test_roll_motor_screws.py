"""roll_motor_screws, the forearm roll motor's 4x M3 SHCS - the elbow and the wrist-pitch motors' screw head and
counterbore (lib/forearm/params.py MotorHoleParams) through the roll frame's thinner plate: a purchased part with no
reference (lib/reference.py NO_REFERENCE), which keeps no reference file - THIS file locks its numbers (the pieces, the
modelled volume and the bounding box, measured on the build and written in) and what it is: every screw on one of the
plate's holes about the motor's axis with its head's bearing face on z=0, as long under the head as the plate under the
counterbores and the thread into the motor. The plate (RollDriveParams pad_t, motor_holes): round holes - no slide, the
idler tensions the belt -, the counterbores in its front face, probed on the frame. In the module: the heads flush with
the front face, the thread in the motor, the pattern clear of everything else in the drive."""
import pytest
from cadgen.geometry import closest_points

import parts
from assemblies import forearm_roll_drive as M
from lib import params as PARAMS
from lib import reference as R
from lib.forearm import DEFAULT, pad_bolt_points, stack_positions
from lib.upper_arm import DEFAULT as UPPER_ARM
from tests import built
from tests.helpers import interference, is_inside

NAME = "roll_motor_screws"
D, S = DEFAULT.drive, stack_positions(DEFAULT)
H = D.motor_holes
SCREW = H.screw
# (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCK = (4, 483.148, (-18.25, -18.25, -3.0), (36.5, 36.5, 11.0))


def _inside(shape, x: float, y: float, z: float) -> bool:
    return any(is_inside(s, x, y, z) for s in shape.solids())


def test_it_is_a_purchased_part_with_no_reference_in_the_roll_drive():
    mod = parts.load(NAME)
    assert parts.GROUPS[NAME] == "joints" and parts.bought(NAME) and mod.PURCHASE_QTY == len(pad_bolt_points(DEFAULT)) == LOCK[0]
    assert R.NO_REFERENCE[NAME] == f"parts/joints/{NAME}.py:_envelope()" and NAME not in parts.unplaced()
    # one row in the drive, on the motor's axis, turned over (the heads up the roll axis), on the stator
    row = ((S["x_motor"], S["y_motor"], S["z_motor_screws"]), (180.0, 0.0, 0.0))
    assert [(role, pos) for part, role, pos in M.OCCURRENCES if part == NAME] == [(None, row)]
    assert NAME in M.BODIES["stator"]


def test_the_elbow_motors_head_and_counterbore_in_a_thinner_plate():
    """The wrist-pitch motor's MotorHoleParams - the elbow motor's screw, counterbore and thread -, through the plate:
    pad_t thick, 4 under the heads, the counterbores head_h deep - so M3 x 8 (the elbow's and the wrist's are M3 x 12
    through 8 of web). The pattern is the plate's holes about the motor's axis - the 31 mm square."""
    mod, a = parts.load(NAME), UPPER_ARM.arm
    assert H == DEFAULT.holes and mod.SCREW == SCREW == a.motor_screw == PARAMS.M3_SHCS
    assert H.pocket_dia == a.motor_pocket_dia > SCREW.head_dia and H.thread == PARAMS.J1_MOTOR_SCREW_THREAD
    assert D.pad_t - SCREW.head_h == 4.0
    assert mod.LENGTH == PARAMS.ROLL_MOTOR_SCREW_LEN == D.pad_t - SCREW.head_h + H.thread == 8.0
    assert mod.PURCHASE_SPEC == "M3 x 8 socket head cap screw (ISO 4762)"
    steel = PARAMS.STEEL_DENSITY * LOCK[0]
    assert mod.MASS_G == PARAMS.ROLL_MOTOR_SCREWS_MASS_G == pytest.approx(steel * PARAMS.shcs_volume(SCREW, 8.0))
    half = PARAMS.NEMA17_BOLT_SP / 2.0
    assert H.bolt_half == half and sorted(mod.POINTS) == sorted((sx * half, sy * half) for sx in (1, -1) for sy in (1, -1))
    assert sorted(mod.POINTS) == sorted((x - S["x_motor"], y - S["y_motor"]) for x, y in pad_bolt_points(DEFAULT))


def test_the_stations():
    """The counterbores' floor head_h under the front face, the plate pad_t from the motor's face to it; the thread in
    the motor short of its holes' bottom; the 20T's hub 1.5 into the pilot hole (its set screws above the face) and its
    bore on the shaft to its far end."""
    assert S["z_motor_screws"] == pytest.approx(S["z_front"] - SCREW.head_h)
    assert S["z_front"] - S["z_motor_face"] == pytest.approx(D.pad_t)
    tip = S["z_motor_screws"] - PARAMS.ROLL_MOTOR_SCREW_LEN
    assert S["z_motor_face"] - tip == pytest.approx(H.thread) and H.thread < D.motor.bolt_hole_depth
    assert S["z_front"] - S["z_20t"] == pytest.approx(1.5)
    assert S["z_motor_face"] + D.motor.shaft_length - (S["z_20t"] + D.t20_len) >= 2.0


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
    screws, length = built.part(NAME), PARAMS.ROLL_MOTOR_SCREW_LEN
    for x, y in parts.load(NAME).POINTS:
        assert _inside(screws, x, y, length - 0.1) and not _inside(screws, x, y, length + 0.1)
        assert _inside(screws, x + SCREW.d / 2.0 - 0.1, y, length / 2.0) and not _inside(screws, x + SCREW.d / 2.0 + 0.1, y, length / 2.0)
        assert _inside(screws, x + SCREW.head_dia / 2.0 - 0.1, y, -SCREW.head_h + 0.1)          # the head's rim, to its top
        assert not _inside(screws, x, y, -SCREW.head_h + SCREW.socket_depth - 0.1)              # the socket
        assert _inside(screws, x, y, -SCREW.head_h + SCREW.socket_depth + 0.1)


@pytest.mark.slow
def test_the_plate_on_the_frame():
    """Each hole: its counterbore pocket_dia across from the front face down to head_h, the M3 clearance hole on through
    the plate under it, the plate whole round both - no slide along Y; the pilot hole round, the plate whole beyond it
    along X and above it - the shaft's slit runs down from it (tests/forearm/test_roll_drive.py)."""
    block = built.part("forearm_roll_block")
    r, rb, floor = H.pocket_dia / 2.0, H.bolt_dia / 2.0, S["z_motor_screws"]
    z_mid = (S["z_motor_face"] + floor) / 2.0
    for x, y in pad_bolt_points(DEFAULT):
        assert not is_inside(block, x + r - 0.2, y, floor + 0.2) and not is_inside(block, x + r - 0.2, y, S["z_front"] - 0.2)
        assert is_inside(block, x + r - 0.2, y, floor - 0.2) and is_inside(block, x + r + 0.2, y, S["z_front"] - 0.2)
        assert not is_inside(block, x + rb - 0.2, y, z_mid) and not is_inside(block, x, y, S["z_motor_face"] + 0.2)
        for dy in (r + 0.2, -(r + 0.2)):
            assert is_inside(block, x, y + dy, S["z_front"] - 0.2), (x, y, dy)
        for dy in (rb + 0.2, -(rb + 0.2)):
            assert is_inside(block, x, y + dy, z_mid), (x, y, dy)
    rp, xm, ym = H.pilot_dia / 2.0, S["x_motor"], S["y_motor"]
    assert not is_inside(block, xm - rp + 0.2, ym, z_mid) and is_inside(block, xm - rp - 0.2, ym, z_mid)
    for dx in (rp + 0.2, -(rp + 0.2)):
        assert is_inside(block, xm + dx, ym, z_mid)
    assert is_inside(block, xm, ym + rp + 0.2, z_mid) and not is_inside(block, xm, ym - rp - 0.2, z_mid)


@pytest.mark.slow
def test_in_place_the_heads_are_flush_and_clear_of_the_drive():
    """The placed pattern in the module: the heads' tops on the front face, the heads on the counterbores' floors, the
    tips thread under the motor's face; no interference with anything else in the drive."""
    module = built.model(M.forearm_roll_drive)
    screws = next(c for c in module.children if c.label == NAME)
    bb = screws.bounding_box()
    assert bb.max.Z == pytest.approx(S["z_front"], abs=1e-6) and bb.min.Z == pytest.approx(S["z_motor_face"] - H.thread, abs=1e-6)
    block = next(c for c in module.children if c.label == "forearm_roll_block")
    assert closest_points(screws, block).distance < 0.01
    for other in module.children:
        if other is not screws:
            assert interference(screws, other) == pytest.approx(0.0, abs=1e-6), other.label
