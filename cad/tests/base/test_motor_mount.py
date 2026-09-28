"""base_motor_mount, the base_yaw motor's bolt-on box (lib/base/body.py build_motor_mount, DEFAULT's JointParams +
MountParams): room round the motor's board for the wiring, the window between the base's posts open into the base;
the joint - the two parts meet at split_x without overlapping, flush on the plate's top and the bottom face, 4x M4
through the mount's ears and the base's posts from heads turned outside into nuts pressed into the posts, the tips past
the nuts - and the motor's slotted seat where the stock belt puts the motor: the motor and its board clear the mount
at both ends of the slots' travel."""
import pytest
from build123d import Location

import parts
from assemblies._occurrences import place_world
from lib import mounts
from lib.base import DEFAULT, LEGACY, joint_bolt_points, joint_stations, motor_holes, mount_inner_half, mount_x1
from lib.base.params import MOTOR_TRAVEL, YAW_BELT
from lib.belts import GT2_PULLEY_20T_TEETH, GT2_PULLEY_90T_TEETH, STANDARD_2GT_LENGTHS, closed_belt_length
from lib.datum import BASE_BOTTOM_Y
from lib.fasteners import M4_PITCH
from lib.geom import hex_circumdiameter
from lib.motors import MKS_SERVO42D_W, NEMA17_FACE, NEMA17_PILOT_DIA
from tests.helpers import interference, is_inside

J, M, S, PL, MT = DEFAULT.joint, DEFAULT.motor, DEFAULT.shell, DEFAULT.plate, DEFAULT.mount
ST = joint_stations()
W_IN = mount_inner_half()             # the mount's inside, and the window between the base's posts (|z|)
W_OUT = W_IN + MT.wall                # the mount's side walls' outside
X1 = mount_x1()                       # the mount's end wall's outside


def _belt(x: float) -> float:
    return closed_belt_length(x, GT2_PULLEY_90T_TEETH, GT2_PULLEY_20T_TEETH)


def test_the_motor_sits_where_the_stock_belt_puts_it():
    assert YAW_BELT in STANDARD_2GT_LENGTHS and M.travel == MOTOR_TRAVEL > 0.0
    assert _belt(M.centre[0]) == pytest.approx(YAW_BELT, abs=1e-4)
    assert _belt(LEGACY.motor.centre[0]) == pytest.approx(274.2, abs=0.05)   # the SolidWorks centre: no stock belt
    assert _belt(M.centre[0] - M.travel) < YAW_BELT - 4.0 and _belt(M.centre[0] + M.travel) > YAW_BELT + 4.0   # slack / tension


def test_the_seat_follows_the_slots():
    """The pilot slides in the window, the whole motor stays on the mount at the slots' near end; the slots keep a web
    to the window."""
    c, cz = M.centre
    assert M.window_x[0] < c - M.travel - NEMA17_PILOT_DIA / 2.0 and M.window_x[1] > c + M.travel + NEMA17_PILOT_DIA / 2.0
    assert M.window_half_z > NEMA17_PILOT_DIA / 2.0
    assert c - M.travel - NEMA17_FACE / 2.0 - J.split_x > 1.0
    for _, z in motor_holes():
        assert abs(z - cz) - M.hole_dia / 2.0 - M.window_half_z >= 2.0   # the web between a slot and the window


def test_the_mount_leaves_room_for_the_wiring():
    """MountParams.room clear of the MKS board's square in Z at any travel, and to the end wall at the slots' middle
    (less the travel at its far end); narrower than the base, its ears on the base's end face."""
    c, cz = M.centre
    half = MKS_SERVO42D_W / 2.0
    assert W_IN - (abs(cz) + half) == pytest.approx(MT.room)
    assert X1 - MT.wall - (c + half) == pytest.approx(MT.room)
    assert X1 - MT.wall - (c + M.travel + half) == pytest.approx(MT.room - M.travel)
    assert W_OUT + MT.ear_w < S.r and W_OUT < S.r - 15.0


def test_joint_layout():
    """4 bolts on the ears' centrelines, inside the posts; the heads on the ears clear of the mount's walls; heads and
    nut pockets 2 mm inside the plate's underside and the bottom face; the nut's pocket short of the joint face; each
    tip 2 pitches past its nut."""
    pts = joint_bolt_points()
    z = W_OUT + MT.ear_w / 2.0
    assert len(pts) == 4 and {round(abs(pz), 6) for pz, _ in pts} == {round(z, 6)}
    corner = hex_circumdiameter(J.nut_pocket_af) / 2.0     # a corner up: the pocket's half height
    assert J.screw.head_dia < MT.ear_w and z - J.screw.head_dia / 2.0 - W_OUT >= 2.0           # the key's way past the wall
    assert z - J.nut_pocket_af / 2.0 - W_IN >= 2.0 and z + J.nut_pocket_af / 2.0 < S.r         # the pocket in the post
    half = max(corner, J.screw.head_dia / 2.0)
    for _, y in pts:
        assert PL.y[0] - y - half >= 2.0 and y - half - S.y0 >= 2.0
        assert min(PL.y[0] - y, y - S.y0) == pytest.approx(J.bolt_inset)
    assert J.post_t - J.nut.h >= 2.0 and ST["x_nut_face"] < ST["x_split"]
    assert ST["x_post"] - ST["x_tip"] >= 2.0 * M4_PITCH


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
    assert bm.max.X == pytest.approx(X1, abs=1e-6) and (bm.min.Z, bm.max.Z) == pytest.approx((-W_OUT - MT.ear_w, W_OUT + MT.ear_w), abs=1e-6)
    assert bm.min.Y == pytest.approx(BASE_BOTTOM_Y, abs=1e-6) == bb.min.Y
    assert bm.max.Y == pytest.approx(PL.y[1], abs=1e-6)     # the plate's top and the walls' tops: one face, printed down
    assert interference(base, mount) < 1e-3
    x_base, x_mount = J.split_x - 0.5, J.split_x + 0.5
    for sz in (1, -1):
        assert is_inside(base, x_base, -70, sz * (W_IN + 0.5)) and not is_inside(base, x_base, -70, sz * (W_IN - 0.5))   # a post
        assert is_inside(mount, x_mount, -70, sz * (W_OUT + MT.ear_w - 0.5))                                         # an ear
        assert not is_inside(mount, x_mount, -70, sz * (W_OUT + MT.ear_w + 0.5))
        assert is_inside(mount, 90, -70, sz * (W_IN + 0.5)) and not is_inside(mount, 90, -70, sz * (W_OUT + 0.5))      # a side wall
    assert not is_inside(base, x_base, -70, 0) and not is_inside(base, x_base, -99, 0)   # the window into the base, full height
    assert not is_inside(mount, 90, -70, 0) and not is_inside(mount, x_mount, -99, 0)    # the mount's inside, open underneath
    for part, x in ((base, x_base), (mount, x_mount)):
        for z, y in joint_bolt_points():
            assert not is_inside(part, x, y, z + J.bolt_dia / 2.0 - 0.1) and is_inside(part, x, y, z + J.bolt_dia / 2.0 + 0.1)
    assert is_inside(base, J.split_x - 2.0, -42, 0) and is_inside(mount, J.split_x + 2.0, -42, 0)   # the plate whole: no belt slot
    assert is_inside(mount, X1 - MT.wall / 2.0, -90, 0) and is_inside(mount, X1 - MT.wall / 2.0, -60, 0)   # the end wall: no notch


@pytest.mark.slow
def test_the_base_holds_the_nuts(base):
    """Each nut's hex pocket from the post's back face to the nut's bearing face, a corner up; the post beyond it."""
    x_in = (ST["x_post"] + ST["x_nut_face"]) / 2.0
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
    for x, z in ((c + M.travel + NEMA17_FACE / 2.0 + 1.3, cz), (c, cz + NEMA17_FACE / 2.0 + 1.3)):
        assert not is_inside(mount, x, PL.y[0] - 3.0, z)                                              # no rim under the plate


@pytest.mark.slow
def test_the_screws_clamp_the_joint():
    """Every head's bearing face on an ear's outer face, every tip 2 pitches past its nut's outer face, into the base's
    cavity below the plate; each nut coaxial with its screw."""
    screws = place_world("base_motor_mount_screws", "base_motor_mount_screws#1")
    nuts = place_world("base_motor_mount_nuts", "base_motor_mount_nuts#1")
    for solid in screws.solids():
        bb = solid.bounding_box()
        assert bb.max.X == pytest.approx(ST["x_head"] + J.screw.head_h, abs=1e-4)
        assert bb.min.X == pytest.approx(ST["x_tip"], abs=1e-4) and ST["x_post"] - bb.min.X >= 2.0 * M4_PITCH
        assert bb.max.Y < PL.y[0]
    assert sorted((round(n.bounding_box().min.X, 4), round(n.bounding_box().max.X, 4)) for n in nuts.solids()) == [
        (round(ST["x_post"], 4), round(ST["x_nut_face"], 4))] * 4


@pytest.mark.slow
@pytest.mark.parametrize("dx", [-MOTOR_TRAVEL, MOTOR_TRAVEL], ids=["slack", "tension"])
def test_the_motor_clears_the_mount_across_the_travel(mount, dx):
    """The 48 mm motor and its board slid to either end of the slots: nothing runs into the mount (the motor's face on
    the plate's underside, the pilot in the window), the board the wiring room clear of it, less the travel."""
    shift = Location((dx, 0.0, 0.0))
    for key, clear in (("nema17_48mm#1", 0.0), ("mks_servo42d#1", MT.room - M.travel - 0.1)):
        part = place_world(mounts.BY_KEY[key].part, key).moved(shift)
        assert interference(part, mount) < 1e-3, key
        if clear:
            assert part.distance_to(mount) >= clear, key
