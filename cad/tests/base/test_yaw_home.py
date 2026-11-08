"""The base_yaw home sensor (lib/base/params.py YawHomeParams): a KY-003 hall module on a printed bracket hung under the
base's motor plate, its chip under the 120T's outer face, and a Ø6 x 3 N52 magnet in a pocket in that face
(lib/pulley/params.py YAW magnet) that stands over the chip at BASE_YAW_HOME_DEG (lib/params.py).

The home angle is where the switch's zone sits: base_yaw turns +/- BASE_YAW_LIMIT_DEG, so its two ends are only
360 - 2 x the limit apart round the circle; the zone must run on past the - end (the arm can never sit between it and
that end, so homing - always toward - never runs past the end) and stop short of the + end (which must never read as
home), for every operate point the A3144 may have (lib/sensors.py A3144_OPERATE_MT, switch_half_width()). The bracket,
the magnet and the fasteners have no reference (lib/reference.py NO_REFERENCE), so THIS file locks their numbers. The
layout, the seat, the pocket, the inserts' holes and the fit are probed; the magnet stands over the chip at home."""
import math

import pytest
from build123d import Axis, Location
from cadgen.build123d import closest_points

import parts
from lib import params as PARAMS
from lib import placements as P
from lib import reference as R
from lib.base import (
    DEFAULT,
    LEGACY,
    home_board_points,
    home_board_y,
    home_bracket_frame,
    home_chip_xz,
    home_outline,
    home_pad_box,
    home_reliefs,
    home_screw_points,
    home_sensor_frame,
    home_to_base,
    inner_r,
)
from lib.base.params import YAW_BELT_Y
from lib.belts import GT2_BELT_W
from lib.fasteners import M2_SHCS, M3_PITCH, heat_set_insert, insert_volume, nut_volume, shcs_volume
from lib.pulley.params import YAW, YAW_MAGNET_DEG, YAW_MAGNET_R
from lib.sensors import (
    A3144_BODY_T,
    A3144_HALL_DEPTH,
    A3144_LEAD_PITCH,
    A3144_OPERATE_MT,
    HOME_MAGNET_DIA,
    HOME_MAGNET_T,
    KY003_BOARD_L,
    KY003_BOARD_T,
    KY003_BOARD_W,
    KY003_HEADER_H,
    KY003_LEAD_TAIL,
    KY003_PIN_OUT,
    KY003_PIN_PITCH,
    KY003_PIN_TAIL,
    MAGNET_POCKET_CLEAR,
    disc_magnet_bz,
    ky003_hall_point,
    switch_half_width,
)
from lib.yaw_coupler.layout import yaw_magnet_deg, yaw_pulley_frame
from tests import built
from tests.helpers import interference, is_inside

H, PL, CAP = DEFAULT.home, DEFAULT.plate, DEFAULT.cap
FACE_Y = CAP.ring_top_y + yaw_pulley_frame()[0][1] - YAW.face_y   # the 120T's outer face in the base's frame (-29.3): the
#                                                                   coupler's origin at the ring's top, the pulley flipped
CHIP_TOP = home_board_y() + KY003_BOARD_T + A3144_BODY_T          # the chip's branded face
HALL_Y = CHIP_TOP - A3144_HALL_DEPTH                              # its hall element
MARGIN = 2.5          # deg: the zone's far edge this far past the - end and short of the + end, for every operate point
PLUG_LEN = 14.0       # [ESTIMATE] a 3-pin Dupont housing on the header's pins, from the header's face
BOX_CLEAR = 0.5       # the board's footprint, its pins and the plug this far in from the plate's opening where it is there
# name -> (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCKS = {
    "yaw_home_bracket": (1, 3906.738, (-23.0, -55.0, 0.0), (43.0, 37.0, 6.0)),
    "yaw_home_magnet": (1, 84.823, (-3.0, -3.0, 0.0), (6.0, 6.0, 3.0)),
    "yaw_home_screws": (2, 213.300, (3.25, -52.25, -3.0), (14.5, 5.5, 9.0)),
    "yaw_home_inserts": (2, 76.404, (3.7, -51.8, 3.0), (13.6, 4.6, 4.0)),
    "yaw_home_board_screws": (2, 104.299, (-7.41, -33.9, -2.4), (3.8, 13.8, 12.0)),
    "yaw_home_board_nuts": (2, 34.287, (-7.51, -34.309401, -1.6), (4.0, 14.618802, 1.6)),
}
SENSOR = ("ky003_hall_sensor#1", "yaw_home_bracket#1", "yaw_home_screws#1", "yaw_home_inserts#1", "yaw_home_board_screws#1",
          "yaw_home_board_nuts#1")


def _deg(mm: float) -> float:
    """A length along the chip's circle as degrees of base_yaw."""
    return math.degrees(mm / H.chip_r)


def _in_opening(x: float, z: float, clear: float = BOX_CLEAR) -> bool:
    """Is (x, z) clear of the motor plate: inside its central opening, or in the sector where it runs out to the wall?"""
    deg = math.degrees(math.atan2(z, x)) % 360.0
    return PL.open_deg[0] <= deg <= PL.open_deg[1] or math.hypot(x, z) <= PL.opening_r - clear


def test_the_home_angle():
    """The magnet's pocket on the chip's radius; mounted, it stands over the chip at BASE_YAW_HOME_DEG, inside the travel
    and near its - end; LEGACY has no sensor."""
    assert YAW.magnet[:2] == (YAW_MAGNET_R, YAW_MAGNET_DEG) and H.chip_r == YAW_MAGNET_R and LEGACY.home is None
    assert YAW.magnet[2:] == pytest.approx((HOME_MAGNET_DIA + 2.0 * MAGNET_POCKET_CLEAR, HOME_MAGNET_T + MAGNET_POCKET_CLEAR))
    assert PARAMS.BASE_YAW_HOME_DEG == pytest.approx(yaw_magnet_deg() - H.chip_deg) == pytest.approx(-169.5)
    assert -PARAMS.BASE_YAW_LIMIT_DEG < PARAMS.BASE_YAW_HOME_DEG < -PARAMS.BASE_YAW_LIMIT_DEG + 10.0
    assert home_chip_xz() == pytest.approx((0.0, H.chip_r))


def test_the_switch_zone_fits_between_the_ends():
    """For both ends of the A3144's operate point: a strong field at the chip, and the switch's zone from past the - end
    to short of the + end (360 - 2 x the limit round the circle), MARGIN to spare both ways."""
    gap = FACE_Y - HALL_Y
    assert gap == pytest.approx(3.48, abs=0.01)
    assert disc_magnet_bz(HOME_MAGNET_DIA, HOME_MAGNET_T, 0.0, gap) >= 3.0 * max(A3144_OPERATE_MT)
    home, limit = PARAMS.BASE_YAW_HOME_DEG, PARAMS.BASE_YAW_LIMIT_DEG
    half = [_deg(switch_half_width(HOME_MAGNET_DIA, HOME_MAGNET_T, gap, mt)) for mt in A3144_OPERATE_MT]
    assert half == pytest.approx([12.8, 8.5], abs=0.1)              # the most sensitive chip's zone, the least's
    assert home - min(half) <= -limit - MARGIN                     # past the - end ...
    assert home - max(half) >= -(360.0 - limit) + MARGIN           # ... short of the + end, round the circle
    assert home + max(half) < 0.0                                  # the edge homing stops at: well inside the travel


def test_the_layout():
    """The board on the pad, the chip 2.98 under the 120T's face, the header 1.96; the board, its pins and the plug clear
    of the plate and under the belt; the M3s under the plate - clear of its opening, its arc slot and the wall -, each in
    its insert, 1.0 of plate over the insert; the M2s through the board's holes, 2 pitches past their nuts; the pad's
    slots under the pins' and the leads' tails."""
    assert home_board_y() == pytest.approx(PL.y[0] + H.pad_h)
    assert FACE_Y == pytest.approx(-29.3) and FACE_Y - CHIP_TOP == pytest.approx(2.98)
    assert FACE_Y - (home_board_y() + KY003_BOARD_T + KY003_HEADER_H) == pytest.approx(1.96)
    (ox, oy, oz), _ = home_sensor_frame()
    hx, hy, _ = ky003_hall_point()
    assert (ox - hx, oz + hy) == pytest.approx(home_chip_xz()) and oy == home_board_y()
    reach = KY003_BOARD_L + KY003_PIN_OUT + PLUG_LEN
    for x, y in ((0.0, KY003_BOARD_W / 2.0), (0.0, -KY003_BOARD_W / 2.0), (KY003_BOARD_L, KY003_BOARD_W / 2.0),
                 (KY003_BOARD_L, -KY003_BOARD_W / 2.0), (reach, 1.5 * KY003_PIN_PITCH), (reach, -1.5 * KY003_PIN_PITCH)):
        assert _in_opening(*home_to_base(DEFAULT, x, y))
    plug_top = home_board_y() + KY003_BOARD_T + KY003_HEADER_H / 2.0 + KY003_PIN_PITCH / 2.0
    assert YAW_BELT_Y - GT2_BELT_W / 2.0 - plug_top >= 2.0
    ins = H.insert
    assert PL.y[1] - PL.y[0] - ins.length == pytest.approx(1.0) and ins.hole_dia < ins.od
    for x, z in home_screw_points():
        r, deg = math.hypot(x, z), math.degrees(math.atan2(z, x))
        assert r - ins.od / 2.0 >= PL.opening_r + 3.0 and PL.open_deg[1] - 360.0 < deg < PL.open_deg[0]
        assert abs(deg) - PL.arc_slot_half_deg >= 20.0
        assert z + ins.od / 2.0 <= inner_r(DEFAULT, PL.y[0]) - 5.0
    tip = home_bracket_frame()[0][1] + H.screw_len
    assert PL.y[0] + ins.length - tip == pytest.approx(1.0) and tip - PL.y[0] >= 6 * M3_PITCH
    top = H.bracket_t + H.pad_h + KY003_BOARD_T
    assert H.board_screw_len - top - H.board_nut.h == pytest.approx(0.8) and H.board_hole_dia > M2_SHCS.d
    x0, x1, z0, z1 = home_pad_box()
    assert all(x0 < x < x1 and z0 < z < z1 for x, z in home_board_points())
    (pin_x0, pin_x1, pin_z0, pin_z1, pin_d), (lead_x0, lead_x1, lead_z0, lead_z1, lead_d) = home_reliefs()
    assert pin_d == pytest.approx(KY003_PIN_TAIL + H.relief_past) and lead_d == pytest.approx(KY003_LEAD_TAIL + H.relief_past)
    assert pin_d > H.pad_h and pin_d - H.pad_h < H.bracket_t        # the pins' slot into the plate, not through it
    assert pin_z1 - pin_z0 > 2.0 * KY003_PIN_PITCH and lead_z1 - lead_z0 > 2.0 * A3144_LEAD_PITCH
    assert x0 < pin_x0 < pin_x1 < lead_x0 < lead_x1 < x1
    outline = home_outline()
    xs, zs = zip(*outline, strict=True)
    assert min(xs) < x0 and max(zs) > max(z for _, z in home_screw_points())


def test_the_parts_have_no_reference():
    for name, (pieces, volume, lo, size) in LOCKS.items():
        mod, shape = parts.load(name), parts.build(name)
        assert parts.GROUPS[name] == "base" and name not in parts.unplaced() and len(shape.solids()) == pieces
        assert parts.bought(name) == (name != "yaw_home_bracket")
        assert R.NO_REFERENCE[name].startswith(("parts/base/", "lib/base/body.py"))
        assert R.solid_volume(shape) == pytest.approx(volume, abs=1e-3)
        bb = shape.bounding_box()
        assert tuple(bb.min) == pytest.approx(lo, abs=1e-6) and tuple(bb.size) == pytest.approx(size, abs=1e-6)
        if parts.bought(name):
            assert mod.PURCHASE_QTY == pieces
    assert "ky003_hall_sensor" not in parts.unplaced()
    assert parts.load("yaw_home_magnet").MASS_G == pytest.approx(PARAMS.NDFEB_DENSITY * LOCKS["yaw_home_magnet"][1], abs=1e-3)
    assert parts.load("yaw_home_screws").MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * 2 * shcs_volume(H.screw, H.screw_len))
    assert parts.load("yaw_home_inserts").MASS_G == pytest.approx(PARAMS.BRASS_DENSITY * 2 * insert_volume(H.insert))
    assert parts.load("yaw_home_board_nuts").MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * 2 * nut_volume(H.board_nut))
    assert insert_volume(H.insert) * 2 == pytest.approx(LOCKS["yaw_home_inserts"][1], abs=1e-3)
    assert heat_set_insert(H.insert).volume == pytest.approx(insert_volume(H.insert), abs=1e-6)


@pytest.mark.slow
def test_the_pocket_in_the_120t():
    """The 120T's pocket, from its outer face into the web: open at its middle down its depth, the web whole past it and
    round it - clear of the rim and the bolts' counterbores."""
    pulley = built.part("gt2_pulley_120t")
    r, deg, dia, depth = YAW.magnet
    x, z = r * math.cos(math.radians(deg)), r * math.sin(math.radians(deg))
    for y in (YAW.face_y - 0.2, YAW.face_y - depth + 0.2):
        assert not is_inside(pulley, x, y, z)
    assert is_inside(pulley, x, YAW.face_y - depth - 0.3, z)
    for a in range(0, 360, 45):
        ex, ez = x + (dia / 2.0 + 0.4) * math.cos(math.radians(a)), z + (dia / 2.0 + 0.4) * math.sin(math.radians(a))
        assert is_inside(pulley, ex, YAW.face_y - depth / 2.0, ez)
    assert r + dia / 2.0 <= YAW.rim_dia / 2.0 - 5.0
    for bx, bz in YAW.bolt_points():
        assert math.hypot(x - bx, z - bz) - dia / 2.0 - YAW.counterbore[0] / 2.0 >= 10.0


@pytest.mark.slow
def test_the_seat_and_the_holes():
    """The bracket: its plate under the board, the pad's top level with the board's underside, the relief slots open to
    their depth, the M3 and M2 holes open; the base: the inserts' holes up into the plate's underside, solid over them."""
    bracket, base = built.part("yaw_home_bracket"), built.part("base")
    t, top = H.bracket_t, H.bracket_t + H.pad_h
    x0, x1, z0, z1 = home_pad_box()
    for x, z in ((x0 + 0.5, z0 + 0.5), (x1 - 0.5, z1 - 0.5)):
        assert is_inside(bracket, x, -z, top - 0.2) and not is_inside(bracket, x, -z, top + 0.2)
    for rx0, rx1, rz0, rz1, depth in home_reliefs():
        x, z = (rx0 + rx1) / 2.0, (rz0 + rz1) / 2.0
        assert not is_inside(bracket, x, -z, top - depth + 0.2) and is_inside(bracket, x, -z, top - depth - 0.2)
    for x, z in home_screw_points():
        assert not is_inside(bracket, x, -z, t / 2.0)
        assert is_inside(bracket, x + H.screw_hole_dia / 2.0 + 0.4, -z, t / 2.0)
        y0 = PL.y[0]
        assert not is_inside(base, x, y0 + H.insert.length - 0.2, z) and is_inside(base, x, y0 + H.insert.length + 0.3, z)
        assert is_inside(base, x + H.insert.hole_dia / 2.0 + 0.4, y0 + 1.0, z)
    for x, z in home_board_points():
        assert not is_inside(bracket, x, -z, top - 0.5) and not is_inside(bracket, x, -z, 0.5)


@pytest.mark.slow
def test_the_sensor_fits():
    """In the base's frame: the bracket on the plate's underside, the board on the pad, the screws' heads on the board
    and under the bracket, the nuts under it - touching, nothing overlapping but the inserts' knurl pressed into the
    plate; the KY-003 1.96 under the 120T's face, well off the belt; nothing of it near the 120T's screws."""
    into = "base#1"
    base = built.placed("base#1", into=into)
    ky, bracket, screws, inserts, board_screws, nuts = (built.placed(k, into=into) for k in SENSOR)
    magnet = built.placed("yaw_home_magnet#1", into=into)
    turning = [built.placed(k, into=into) for k in ("gt2_pulley_120t#1", "yaw_pulley_screws#1")]
    near = [base, built.placed("yaw_belt#1", into=into), built.placed("bearing_6806#2", into=into), *turning, magnet]
    kit = [ky, bracket, screws, inserts, board_screws, nuts]
    press = math.pi * ((H.insert.od / 2.0) ** 2 - (H.insert.hole_dia / 2.0) ** 2) * H.insert.length * 2
    assert interference(inserts, base) == pytest.approx(press, rel=0.01)
    for part in kit:
        for other in near + kit:
            if other is part or {part, other} == {inserts, base}:
                continue
            assert interference(part, other) < 0.01
    assert closest_points(bracket, base).distance < 0.01 and closest_points(ky, bracket).distance < 0.01
    assert closest_points(board_screws, ky).distance < 0.01 and closest_points(nuts, bracket).distance < 0.01
    assert closest_points(screws, bracket).distance < 0.01 and closest_points(inserts, screws).distance < 0.01
    assert closest_points(ky, turning[0]).distance == pytest.approx(1.96, abs=0.01)
    assert closest_points(ky, turning[1]).distance > 5.0 and closest_points(ky, near[1]).distance > 3.0
    for k in (bracket, screws, board_screws):
        assert closest_points(k, turning[0]).distance > 1.9


@pytest.mark.slow
def test_the_magnet_stands_over_the_chip_at_home():
    """In the capture frame W (base_yaw turns about its +Y): the magnet in its pocket, flush with the 120T's face; at
    BASE_YAW_HOME_DEG its axis runs through the hall element, its south pole's face 3.48 over it; at zero it is a
    quarter turn and more away."""
    hall = (P.location("ky003_hall_sensor#1", "world") * Location(ky003_hall_point())).position
    magnet = P.location("yaw_home_magnet#1", "world")
    pulley = built.placed("gt2_pulley_120t#1")
    assert interference(built.placed("yaw_home_magnet#1"), pulley) < 1e-3
    assert magnet.position.Y == pytest.approx(FACE_Y) and hall.Y == pytest.approx(HALL_Y)
    home = Location((0.0, 0.0, 0.0), (0.0, PARAMS.BASE_YAW_HOME_DEG, 0.0)) * magnet
    axis = Axis(home.position, home.z_axis.direction)
    assert axis.direction.Y == pytest.approx(1.0)                        # into the pulley, up: its pole face down
    assert math.hypot(home.position.X - hall.X, home.position.Z - hall.Z) < 1e-6
    assert home.position.Y - hall.Y == pytest.approx(3.48, abs=0.01)
    assert math.hypot(magnet.position.X - hall.X, magnet.position.Z - hall.Z) > H.chip_r
