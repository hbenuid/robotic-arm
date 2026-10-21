"""base_motor_mount, the base_yaw motor's bolt-on half of the base (lib/base/body.py build_motor_mount, DEFAULT's
JointParams + MountParams): the base's D carried on at full width to a round end that mirrors the base's round half, on
the same foot with its own screw holes, the window between the two parts' posts open into the base, room round the
motor's board for the wiring; the slits through both parts' walls (SlitParams: one band, upright, round-ended, none at
the joint's posts); the joint - the two parts meet at split_x without overlapping, flush on the walls, the plate's
top, the foot and the bottom face, 4x M4 through the mount's posts and the base's from heads inside the mount into
nuts pressed into the base's posts, the tips past the nuts - and the motor's seat on round holes YAW_MOTOR_X out, short
of the stock belt's own centre distance (its idler takes up the rest - tests/base/test_yaw_idler.py): the motor and its
board clear the mount."""
import itertools
import math

import pytest

from lib.base import (
    DEFAULT,
    LEGACY,
    flare_points,
    inner_r,
    joint_bolt_points,
    joint_stations,
    motor_face_y,
    motor_holes,
    motor_screw_y,
    mount_end_x,
    mount_foot_holes,
    mount_inner_half,
    mount_x1,
    outer_r,
    slit_band,
    slits,
)
from lib.base.params import YAW_BELT, YAW_MOTOR_PAD, YAW_MOTOR_X
from lib.belts import (
    GT2_PULLEY_20T_TEETH,
    GT2_PULLEY_90T_TEETH,
    GT2_PULLEY_120T_TEETH,
    STANDARD_2GT_LENGTHS,
    closed_belt_length,
)
from lib.datum import BASE_BOTTOM_Y
from lib.fasteners import M4_PITCH
from lib.forearm import DEFAULT as FOREARM
from lib.geom import hex_circumdiameter
from lib.motors import MKS_SERVO42D_W, NEMA17_FACE, NEMA17_PILOT_DIA
from tests import built
from tests.helpers import interference, is_inside

J, M, S, PL, MT, F, SL = DEFAULT.joint, DEFAULT.motor, DEFAULT.shell, DEFAULT.plate, DEFAULT.mount, DEFAULT.foot, DEFAULT.slits
ST = joint_stations()
W_IN = mount_inner_half()             # the window between the two parts' posts (|z|)
CX = mount_end_x()                    # the mount's end round's centre
BAND = slit_band()


def _on_a_side(x: float, deg: float) -> bool:
    """A slit on a straight side (between the two round ends), not on a round end."""
    return abs(abs(deg) - 90.0) < 1e-9 and 1e-6 < x < CX - 1e-6


def _belt(x: float, teeth: int = GT2_PULLEY_120T_TEETH) -> float:
    return closed_belt_length(x, teeth, GT2_PULLEY_20T_TEETH)


def test_the_motor_sits_short_of_the_stock_belt():
    """On round holes YAW_MOTOR_X out, short of the stock belt's own centre distance round the base_yaw 120T: the idler
    takes up the rest (tests/base/test_yaw_idler.py)."""
    assert YAW_BELT in STANDARD_2GT_LENGTHS and M.travel == 0.0 and M.centre[0] == YAW_MOTOR_X
    assert _belt(M.centre[0]) < YAW_BELT - 5.0                                                     # round the base_yaw 120T
    assert _belt(LEGACY.motor.centre[0], GT2_PULLEY_90T_TEETH) == pytest.approx(274.2, abs=0.05)   # the SolidWorks centre (a 90T): no stock belt


def test_the_seat():
    """The wrist and roll motors' round seat (MotorParams.holes): the pilot in its round hole (the 20T's hub too, which
    dips into the plate: test_the_motor_clears_the_mount), a web between each hole's counterbore and it; the pad under
    the plate the motor's face, the plate and the pad under the heads the roll motor's 4 (the counterbores head_h deep:
    flush); the motor's face wholly on the pad, clear of the posts at the joint face."""
    c, cz = M.centre
    h = M.holes
    assert h == FOREARM.holes == FOREARM.drive.motor_holes and M.hole_dia == h.bolt_dia and M.travel == 0.0
    assert h.pilot_dia - NEMA17_PILOT_DIA == pytest.approx(0.3) and LEGACY.motor.holes is None and LEGACY.motor.pad_t == 0.0
    assert M.pad_t == YAW_MOTOR_PAD == 2.0 and motor_face_y() == pytest.approx(PL.y[0] - M.pad_t)
    assert motor_screw_y() - motor_face_y() == pytest.approx(PL.y[1] - PL.y[0] + M.pad_t - h.screw.head_h) == pytest.approx(4.0)
    assert c - NEMA17_FACE / 2.0 - (J.split_x + MT.post_t) > 2.0
    for x, z in motor_holes():
        assert math.hypot(x - c, z - cz) - h.pocket_dia / 2.0 - h.pilot_dia / 2.0 >= 5.0   # the web between them


def test_the_mount_is_the_bases_other_half():
    """The window between the posts MountParams.room clear of the MKS board's square in Z; the round end room past the
    board's square at the plate's top and the base's round half mirrored - its radius the walls' at every height -, so
    the mount's outline is the base's D carried on; its end past the motor's whole face."""
    c, cz = M.centre
    half = MKS_SERVO42D_W / 2.0
    assert W_IN - (abs(cz) + half) == pytest.approx(MT.room)
    assert mount_x1() - S.wall - (c + half) == pytest.approx(MT.room)
    assert CX + outer_r(DEFAULT, PL.y[1]) == pytest.approx(mount_x1())
    assert J.split_x < CX < c - NEMA17_FACE / 2.0                       # the straight sides run on past the joint
    assert MT.post_t == J.post_t                                         # the posts mirror the base's


def test_joint_layout():
    """4 bolts bolt_out out of the window's edge, through both parts' posts; each head on the mount's post's face, clear
    of the window's edge and of the wall at its height (the key's way along -X inside the mount); heads and nut pockets
    2 mm inside the plate's underside and the bottom face; the nut's pocket short of the joint face; each tip 2 pitches
    past its nut."""
    pts = joint_bolt_points()
    z = W_IN + J.bolt_out
    assert len(pts) == 4 and {round(abs(pz), 6) for pz, _ in pts} == {round(z, 6)}
    corner = hex_circumdiameter(J.nut_pocket_af) / 2.0     # a corner up: the pocket's half height
    assert z - J.nut_pocket_af / 2.0 - W_IN >= 2.0 and z + J.nut_pocket_af / 2.0 < S.r         # the pocket in the post
    half = max(corner, J.screw.head_dia / 2.0)
    for _, y in pts:
        assert z - J.screw.head_dia / 2.0 - W_IN >= 2.0 and inner_r(DEFAULT, y + half) - (z + J.screw.head_dia / 2.0) >= 10.0
        assert PL.y[0] - y - half >= 2.0 and y - half - S.y0 >= 2.0
        assert min(PL.y[0] - y, y - S.y0) == pytest.approx(J.bolt_inset)
    assert J.post_t - J.nut.h >= 2.0 and ST["x_nut_face"] < ST["x_split"] and ST["x_head"] == J.split_x + MT.post_t
    assert ST["x_post"] - ST["x_tip"] >= 2.0 * M4_PITCH


def test_the_mount_foot_holes():
    """The mount's 4 screw holes: the base's round half's, mirrored onto its end round - on hole_r about its centre,
    each spot-face clear of the wall and inside the flange's rim, neighbours a spot-face apart."""
    holes = mount_foot_holes()
    spot = F.spot_dia / 2.0
    assert len(holes) == 4 and sorted(F.mount_hole_deg) == sorted(180.0 - a for a in F.hole_deg)
    for x, z in holes:
        assert math.hypot(x - CX, z) == pytest.approx(F.hole_r) and x - spot > J.split_x
        assert F.hole_r - spot - outer_r(DEFAULT, S.y0) > 8.0 and F.hole_r + spot < F.r
    ring = sorted(holes, key=lambda h: h[1])
    assert min(math.dist(a, b) for a, b in itertools.pairwise(ring)) > 2.0 * F.spot_dia


def test_the_slits():
    """One band, over_flare above the foot's chamfer and under the plate; the base's round half and the mount's round end
    each a slit at both ends and pitch apart along the wall's mid-surface between; the straight sides at multiples of
    pitch, none at the joint's posts; both parts symmetric about z = 0, each slit in its own part; the slit narrower
    than the web between, its round top a w / 2 arch - nothing to bridge."""
    _, (_, yw) = flare_points()
    assert BAND == (yw + SL.over_flare, yw + SL.over_flare + SL.h) and BAND[1] < PL.y[0] - 5.0 and SL.over_flare >= 4.0
    rm = outer_r(DEFAULT, sum(BAND) / 2.0) - S.wall / 2.0
    base, mount = slits(DEFAULT, "base"), slits(DEFAULT, "mount")
    for name, pts, cx, ends in (("base", base, 0.0, (90.0, 270.0)), ("mount", mount, CX, (-90.0, 90.0))):
        rnd = sorted((deg, x, z) for x, z, deg in pts if not _on_a_side(x, deg))
        assert (rnd[0][0], rnd[-1][0]) == pytest.approx(ends) and len(pts) - len(rnd) == {"base": 8, "mount": 4}[name]
        for _deg, x, z in rnd:
            assert math.hypot(x - cx, z) == pytest.approx(rm)
        step = math.radians(rnd[1][0] - rnd[0][0]) * rm
        assert step == pytest.approx(SL.pitch, abs=SL.pitch * 0.05) and step - SL.w > SL.w
        assert sorted((round(x, 6), round(z, 6)) for x, z, _ in pts) == sorted((round(x, 6), round(-z, 6)) for x, z, _ in pts)
    sides = [x for x, z, deg in base + mount if _on_a_side(x, deg)]
    assert all(x / SL.pitch == pytest.approx(round(x / SL.pitch)) for x in sides)
    assert all(abs(x - J.split_x) - SL.w / 2.0 > J.post_t + SL.joint_clear - SL.pitch / 2.0 for x in sides)
    assert not [x for x in sides if J.split_x - J.post_t - SL.w / 2.0 <= x <= J.split_x + MT.post_t + SL.w / 2.0]
    assert all(x < J.split_x for x, _, _ in base) and all(x > J.split_x for x, _, _ in mount)


@pytest.fixture(scope="module")
def base():
    return built.part("base")


@pytest.fixture(scope="module")
def mount():
    return built.part("base_motor_mount")


@pytest.mark.slow
def test_the_parts_meet_at_the_joint(base, mount):
    assert mount.is_valid and len(mount.solids()) == 1
    bb, bm = base.bounding_box(), mount.bounding_box()
    assert bb.max.X == pytest.approx(J.split_x, abs=1e-6) and bm.min.X == pytest.approx(J.split_x, abs=1e-6)
    assert bm.max.X == pytest.approx(CX + F.r, abs=1e-6) and (bm.min.Z, bm.max.Z) == pytest.approx((-F.r, F.r), abs=1e-6)
    assert bm.min.Y == pytest.approx(BASE_BOTTOM_Y, abs=1e-6) == bb.min.Y
    assert bm.max.Y == pytest.approx(PL.y[1], abs=1e-6)     # the plate's top and the walls' tops: one face, printed down
    assert interference(base, mount) < 1e-3
    x_base, x_mount = J.split_x - 0.5, J.split_x + 0.5
    for part, x in ((base, x_base), (mount, x_mount)):
        for sz in (1, -1):
            assert is_inside(part, x, -70, sz * (W_IN + 0.5)) and not is_inside(part, x, -70, sz * (W_IN - 0.5))   # a post
            for y in (S.y0 + F.flare_h + 2.0, -70.0, PL.y[0] - 1.0):      # the walls flush: the same leaning planes
                ro = outer_r(DEFAULT, y)
                assert is_inside(part, x, y, sz * (ro - 0.3)) and not is_inside(part, x, y, sz * (ro + 0.3)), (y, sz)
            assert is_inside(part, x, S.y0 + 1.0, sz * (F.r - 0.5)) and not is_inside(part, x, S.y0 + 1.0, sz * (F.r + 0.5))   # the foot
        for z, y in joint_bolt_points():
            assert not is_inside(part, x, y, z + J.bolt_dia / 2.0 - 0.1) and is_inside(part, x, y, z + J.bolt_dia / 2.0 + 0.1)
    assert not is_inside(base, x_base, -70, 0) and not is_inside(base, x_base, -99, 0)   # the window into the base, full height
    assert not is_inside(mount, 90, -70, 0) and not is_inside(mount, x_mount, -99, 0)    # the mount's inside, open underneath
    assert not is_inside(mount, ST["x_head"] + 1.0, -70, W_IN + J.bolt_out)                # the posts' faces open to the heads
    assert is_inside(base, J.split_x - 2.0, -42, 0) and is_inside(mount, J.split_x + 2.0, -42, 0)   # the plate whole: no belt slot


@pytest.mark.slow
def test_the_round_end_mirrors_the_base(mount):
    """At the end (z = 0, between two slits) and at 45 degrees round it: the wall's outside on the walls' radius about
    the end's centre, its inside `wall` in, below the plate; the foot's flange to foot.r round it, the chamfer's face
    on the base's line; the mount's 4 screw holes through the flange, their spot-faces in the chamfer."""
    for deg in (0.0, 45.0):
        ux, uz = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        for y in (S.y0 + F.flare_h + 2.0, -70.0, PL.y[0] - 1.0):
            ro, ri = outer_r(DEFAULT, y), inner_r(DEFAULT, y)
            assert is_inside(mount, CX + (ro - 0.3) * ux, y, (ro - 0.3) * uz) and not is_inside(mount, CX + (ro + 0.3) * ux, y, (ro + 0.3) * uz)
            assert is_inside(mount, CX + (ri + 0.3) * ux, y, (ri + 0.3) * uz) and not is_inside(mount, CX + (ri - 0.3) * ux, y, (ri - 0.3) * uz)
        yt = S.y0 + F.t
        assert is_inside(mount, CX + (F.r - 0.5) * ux, yt - 0.5, (F.r - 0.5) * uz)
        assert not is_inside(mount, CX + (F.r + 0.5) * ux, yt - 0.5, (F.r + 0.5) * uz)
        (rf, _), (rw, yw) = flare_points()
        n = math.hypot(yw - yt, rf - rw)
        nr, ny = (yw - yt) / n, (rf - rw) / n
        mr, my = (rf + rw) / 2.0, (yt + yw) / 2.0
        assert is_inside(mount, CX + (mr - 0.3 * nr) * ux, my - 0.3 * ny, (mr - 0.3 * nr) * uz)
        assert not is_inside(mount, CX + (mr + 0.3 * nr) * ux, my + 0.3 * ny, (mr + 0.3 * nr) * uz)
    spot, yt = F.spot_dia / 2.0, S.y0 + F.t
    for x, z in mount_foot_holes():
        ux, uz = (CX - x) / F.hole_r, -z / F.hole_r                                      # toward the wall
        assert not is_inside(mount, x, S.y0 + F.t / 2.0, z)                               # the hole
        assert not is_inside(mount, x + (spot - 0.5) * ux, yt + 1.0, z + (spot - 0.5) * uz)   # the spot-face ...
        assert is_inside(mount, x + (spot + 0.5) * ux, yt + 1.0, z + (spot + 0.5) * uz)       # ... in the chamfer
        assert is_inside(mount, x + (spot + 0.5) * ux, yt - 0.5, z + (spot + 0.5) * uz)       # the flange under it


@pytest.mark.slow
def test_the_slits_are_cut(base, mount):
    """Every slit open along its length through the wall, the wall whole beside it and above and below it: probed on the
    wall's mid-surface, which leans out going down (each slit's centre is on it at the band's middle)."""
    ym, h, w = sum(BAND) / 2.0, SL.h, SL.w
    open_dy, shut_dy = h / 2.0 - w / 2.0 - 0.3, h / 2.0 + 0.5
    for part, name in ((base, "base"), (mount, "mount")):
        for x, z, deg in slits(DEFAULT, name):
            nx, nz = math.cos(math.radians(deg)), math.sin(math.radians(deg))
            tx, tz = -nz, nx                                                              # along the wall

            def mid(dy, x=x, z=z, nx=nx, nz=nz):
                d = outer_r(DEFAULT, ym + dy) - outer_r(DEFAULT, ym)
                return x + d * nx, ym + dy, z + d * nz

            for dy in (0.0, open_dy, -open_dy):
                assert not is_inside(part, *mid(dy)), (name, x, z, deg, dy)
            for dy in (shut_dy, -shut_dy):
                assert is_inside(part, *mid(dy)), (name, x, z, deg, dy)
            for s in (1.0, -1.0):
                assert is_inside(part, x + s * (w / 2.0 + 0.4) * tx, ym, z + s * (w / 2.0 + 0.4) * tz), (name, x, z, deg)


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
def test_the_mount_carries_the_seat(mount):
    """The 4 round holes through the plate and its pad, each counterbored from the plate's top (pocket_dia, the screw's
    head_h deep); the pilot's round hole through both; the pad under the plate the motor face's square, no rim round
    it."""
    c, cz = M.centre
    h, face, floor, top = M.holes, motor_face_y(), motor_screw_y(), PL.y[1]
    r, rc, rp = M.hole_dia / 2.0, h.pocket_dia / 2.0, h.pilot_dia / 2.0
    under = (face + floor) / 2.0                                                                      # under the heads
    for x, z in motor_holes():
        assert not is_inside(mount, x, under, z)
        for dx, dz in ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)):
            assert is_inside(mount, x + dx * (r + 0.2), under, z + dz * (r + 0.2))                    # the hole: round
            assert not is_inside(mount, x + dx * (rc - 0.2), top - 0.2, z + dz * (rc - 0.2))          # the counterbore ...
            assert is_inside(mount, x + dx * (rc + 0.2), top - 0.2, z + dz * (rc + 0.2))
        assert not is_inside(mount, x + rc - 0.2, floor + 0.2, z) and is_inside(mount, x + rc - 0.2, floor - 0.2, z)   # ... head_h deep
    for dx, dz in ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)):
        for y in (face + 0.2, top - 0.2):
            assert not is_inside(mount, c + dx * (rp - 0.2), y, cz + dz * (rp - 0.2))                 # the pilot's hole ...
            assert is_inside(mount, c + dx * (rp + 0.2), y, cz + dz * (rp + 0.2))                     # ... round, through
    half = NEMA17_FACE / 2.0
    for dx, dz in ((1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)):
        assert is_inside(mount, c + dx * (half - 0.2), face + 0.2, cz + dz * (half - 0.2))           # the pad ...
        assert not is_inside(mount, c + dx * (half + 0.2), face + 0.2, cz + dz * (half + 0.2))       # ... the face's square
        assert not is_inside(mount, c + dx * (half + 0.2), face - 0.2, cz + dz * (half + 0.2))
    for x, z in ((c + half + 1.3, cz), (c, cz + half + 1.3)):
        assert not is_inside(mount, x, face - 3.0, z)                                                 # no rim under it


@pytest.mark.slow
def test_the_screws_clamp_the_joint():
    """Every head's bearing face on a mount's post's inner face, every tip 2 pitches past its nut's outer face, into the
    base's cavity below the plate; each nut coaxial with its screw."""
    screws = built.placed("base_motor_mount_screws#1")
    nuts = built.placed("base_motor_mount_nuts#1")
    for solid in screws.solids():
        bb = solid.bounding_box()
        assert bb.max.X == pytest.approx(ST["x_head"] + J.screw.head_h, abs=1e-4)
        assert bb.min.X == pytest.approx(ST["x_tip"], abs=1e-4) and ST["x_post"] - bb.min.X >= 2.0 * M4_PITCH
        assert bb.max.Y < PL.y[0]
    assert sorted((round(n.bounding_box().min.X, 4), round(n.bounding_box().max.X, 4)) for n in nuts.solids()) == [
        (round(ST["x_post"], 4), round(ST["x_nut_face"], 4))] * 4


@pytest.mark.slow
def test_the_motor_clears_the_mount(mount):
    """The 48 mm motor and its board on their round holes: nothing runs into the mount (the motor's face on its pad's
    underside, the pilot in its round hole), the board the wiring room clear of it; the joint's screws' heads clear of
    the mount but on their posts; the motor's screws in their holes and counterbores; the 20T's hub, down in the
    pilot's hole, 3 mm clear of it all round."""
    for key, clear in (("nema17_48mm#1", 0.0), ("mks_servo42d#1", MT.room - 0.1), ("base_motor_mount_screws#1", 0.0),
                       ("yaw_motor_screws#1", 0.0), ("gt2_pulley_20t#3", 3.0)):
        part = built.placed(key)
        assert interference(part, mount) < 1e-3, key
        if clear:
            assert part.distance_to(mount) >= clear, key
