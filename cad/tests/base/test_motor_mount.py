"""base_motor_mount, the base's +X lobe as a part of its own (lib/base/body.py build_motor_mount, DEFAULT's
JointParams): the joint - the two parts meet at split_x without overlapping, flush on the plate's top and the bottom
face, 4x M4 through their ribs from heads on the mount into nuts pressed into the base, the tips past the nuts - and
the motor's slotted seat where the stock belt puts the motor: the motor and its board clear the mount at both ends of
the slots' travel."""
import pytest
from build123d import Location

import parts
from assemblies._occurrences import place_world
from lib import mounts
from lib.base import DEFAULT, LEGACY, joint_bolt_points, joint_stations, motor_holes
from lib.base.params import MOTOR_TRAVEL, YAW_BELT
from lib.belts import GT2_PULLEY_20T_TEETH, GT2_PULLEY_90T_TEETH, STANDARD_2GT_LENGTHS, closed_belt_length
from lib.datum import BASE_BOTTOM_Y
from lib.fasteners import M4_PITCH
from lib.geom import hex_circumdiameter
from lib.motors import NEMA17_FACE, NEMA17_PILOT_DIA
from tests.helpers import interference, is_inside

J, M, S, PL = DEFAULT.joint, DEFAULT.motor, DEFAULT.shell, DEFAULT.plate
ST = joint_stations()
RIB_IN = S.r - S.wall - J.rib_w         # the ribs' inside edge (|z|)


def _belt(x: float) -> float:
    return closed_belt_length(x, GT2_PULLEY_90T_TEETH, GT2_PULLEY_20T_TEETH)


def test_the_motor_sits_where_the_stock_belt_puts_it():
    assert YAW_BELT in STANDARD_2GT_LENGTHS and M.travel == MOTOR_TRAVEL > 0.0
    assert _belt(M.centre[0]) == pytest.approx(YAW_BELT, abs=1e-4)
    assert _belt(LEGACY.motor.centre[0]) == pytest.approx(274.2, abs=0.05)   # the SolidWorks centre: no stock belt
    assert _belt(M.centre[0] - M.travel) < YAW_BELT - 4.0 and _belt(M.centre[0] + M.travel) > YAW_BELT + 4.0   # slack / tension


def test_the_seat_follows_the_slots():
    """The pilot slides in the window, the rim's +X side stands past the motor's face at the slots' far end and inside
    the end wall, the whole motor stays on the mount at their near end; the slots keep a web to the window."""
    c, cz = M.centre
    assert M.window_x[0] < c - M.travel - NEMA17_PILOT_DIA / 2.0 and M.window_x[1] > c + M.travel + NEMA17_PILOT_DIA / 2.0
    assert M.window_half_z > NEMA17_PILOT_DIA / 2.0
    assert M.slot_x1 > c + M.travel + NEMA17_FACE / 2.0 and S.x1 - S.wall - (M.slot_x1 + M.rim_wall) > 1.0
    assert c - M.travel - NEMA17_FACE / 2.0 - J.split_x > 1.0
    for _, z in motor_holes():
        assert abs(z - cz) - M.hole_dia / 2.0 - M.window_half_z >= 2.0   # the web between a slot and the window


def test_joint_layout():
    """4 bolts on the ribs' centrelines; heads and nut pockets inside the ribs (2 mm off the plate's underside and the
    bottom face); the nut's pocket short of the joint face; each tip 2 pitches past its nut; the ribs clear of the
    motor."""
    pts = joint_bolt_points()
    assert len(pts) == 4 and {round(abs(z), 6) for z, _ in pts} == {round(RIB_IN + J.rib_w / 2.0, 6)}
    corner = hex_circumdiameter(J.nut_pocket_af) / 2.0     # a corner up: the pocket's half height
    assert J.screw.head_dia < J.rib_w and J.nut_pocket_af < J.rib_w
    half = max(corner, J.screw.head_dia / 2.0)
    for _, y in pts:
        assert PL.y[0] - y - half >= 2.0 and y - half - S.y0 >= 2.0
        assert min(PL.y[0] - y, y - S.y0) == pytest.approx(J.bolt_inset)
    assert J.rib_t - J.nut.h >= 2.0 and ST["x_nut_face"] < ST["x_split"]
    assert ST["x_base_rib"] - ST["x_tip"] >= 2.0 * M4_PITCH
    assert RIB_IN - (abs(M.centre[1]) + NEMA17_FACE / 2.0) > 10.0


@pytest.fixture(scope="module")
def base():
    return parts.build("base")


@pytest.fixture(scope="module")
def mount():
    return parts.build("base_motor_mount")


@pytest.mark.slow
def test_the_parts_meet_at_the_joint(base, mount):
    assert mount.is_valid and len(mount.solids()) == 1
    bb, bm = base.bounding_box(), mount.bounding_box()
    assert bb.max.X == pytest.approx(J.split_x, abs=1e-6) and bm.min.X == pytest.approx(J.split_x, abs=1e-6)
    assert bm.max.X == pytest.approx(S.x1, abs=1e-6) and (bm.min.Z, bm.max.Z) == pytest.approx((-S.r, S.r), abs=1e-6)
    assert bm.min.Y == pytest.approx(BASE_BOTTOM_Y, abs=1e-6) == bb.min.Y
    assert bm.max.Y == pytest.approx(PL.y[1], abs=1e-6)     # the plate's top and the walls' tops: one face, printed down
    assert interference(base, mount) < 1e-3
    for part, x in ((base, J.split_x - 0.5), (mount, J.split_x + 0.5)):
        for sz in (1, -1):
            assert is_inside(part, x, -70, sz * (S.r - 2.5))                                              # the side wall
            assert is_inside(part, x, -70, sz * (RIB_IN + 0.5)) and not is_inside(part, x, -70, sz * (RIB_IN - 0.5))   # the rib
            assert is_inside(part, x, -42, sz * 30)                                                       # the plate
        for z, y in joint_bolt_points():
            assert not is_inside(part, x, y, z + J.bolt_dia / 2.0 - 0.1) and is_inside(part, x, y, z + J.bolt_dia / 2.0 + 0.1)
    assert is_inside(base, J.split_x - 2.0, -42, 0) and not is_inside(mount, J.split_x + 2.0, -42, 0)   # the belt slot: the mount's


@pytest.mark.slow
def test_the_base_holds_the_nuts(base):
    """Each nut's hex pocket from the base rib's back face to the nut's bearing face, a corner up; the rib beyond it."""
    x_in = (ST["x_base_rib"] + ST["x_nut_face"]) / 2.0
    for z, y in joint_bolt_points():
        flat, corner = J.nut_pocket_af / 2.0, hex_circumdiameter(J.nut_pocket_af) / 2.0
        assert not is_inside(base, x_in, y, z + flat - 0.1) and is_inside(base, x_in, y, z + flat + 0.1)          # the flats in Z
        assert not is_inside(base, x_in, y + corner - 0.1, z) and is_inside(base, x_in, y + corner + 0.1, z)      # the corners in Y
        assert is_inside(base, ST["x_nut_face"] + 0.1, y, z + flat - 0.1)                                         # the pocket's floor


@pytest.mark.slow
def test_the_mount_carries_the_slotted_seat(mount):
    c, cz = M.centre
    for x, z in motor_holes():
        r = M.hole_dia / 2.0
        for dx in (-M.travel, 0.0, M.travel):
            assert not is_inside(mount, x + dx, -42, z)                                                   # the slot
        assert is_inside(mount, x + M.travel + r + 0.2, -42, z) and is_inside(mount, x - M.travel - r - 0.2, -42, z)
        assert is_inside(mount, x, -42, z + (r + 0.2) * (1 if z > cz else -1))
    assert not is_inside(mount, c, -42, cz + M.window_half_z - 0.1) and is_inside(mount, c, -42, cz + M.window_half_z + 0.1)
    assert is_inside(mount, M.slot_x1 + 1.0, -48, cz) and not is_inside(mount, M.slot_x1 - 1.0, -48, cz)   # the rim
    assert not is_inside(mount, S.x1 - 2.5, -90, 0) and is_inside(mount, S.x1 - 2.5, -90, 20)             # the cable notch


@pytest.mark.slow
def test_the_screws_clamp_the_ribs():
    """Every head's bearing face on the mount rib's inside face, every tip 2 pitches past its nut's outer face, into the
    base's cavity below the plate; each nut coaxial with its screw."""
    screws = place_world("base_motor_mount_screws", "base_motor_mount_screws#1")
    nuts = place_world("base_motor_mount_nuts", "base_motor_mount_nuts#1")
    for solid in screws.solids():
        bb = solid.bounding_box()
        assert bb.max.X == pytest.approx(ST["x_head"] + J.screw.head_h, abs=1e-4)
        assert bb.min.X == pytest.approx(ST["x_tip"], abs=1e-4) and ST["x_base_rib"] - bb.min.X >= 2.0 * M4_PITCH
        assert bb.max.Y < PL.y[0]
    assert sorted((round(n.bounding_box().min.X, 4), round(n.bounding_box().max.X, 4)) for n in nuts.solids()) == [
        (round(ST["x_base_rib"], 4), round(ST["x_nut_face"], 4))] * 4


@pytest.mark.slow
@pytest.mark.parametrize("dx", [-MOTOR_TRAVEL, MOTOR_TRAVEL], ids=["slack", "tension"])
def test_the_motor_clears_the_mount_across_the_travel(mount, dx):
    """The 48 mm motor and its board slid to either end of the slots: nothing runs into the mount (the motor's face on
    the plate's underside, the pilot in the window), the board 1 mm clear of it."""
    shift = Location((dx, 0.0, 0.0))
    for key, clear in (("nema17_48mm#1", 0.0), ("mks_servo42d#1", 1.0)):
        part = place_world(mounts.BY_KEY[key].part, key).moved(shift)
        assert interference(part, mount) < 1e-3, key
        if clear:
            assert part.distance_to(mount) >= clear, key
