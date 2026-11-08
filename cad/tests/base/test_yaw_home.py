"""The base_yaw home sensor (lib/base/params.py YawHomeParams): a KY-003 hall module on a printed bracket hung under the
base's motor plate, its chip under the 120T, and a Ø6 x 3 N52 magnet glued onto the 120T's outer face at its spot
(lib/pulley/params.py YAW_MAGNET_*) that stands over the chip at BASE_YAW_HOME_DEG (lib/params.py).

The home angle is where the switch's zone sits: base_yaw turns +/- BASE_YAW_LIMIT_DEG, so its two ends are only
360 - 2 x the limit apart round the circle; the zone must run on past the - end (the arm can never sit between it and
that end, so homing - always toward - never runs past the end) and stop short of the + end (which must never read as
home), for every operate point the A3144 may have (lib/sensors.py A3144_OPERATE_MT, switch_half_width()). The bracket,
the magnet and the fasteners have no reference (lib/reference.py NO_REFERENCE), so THIS file locks their numbers. The
layout, the bracket's slots and holes, the plate's nut pockets and the fit are probed; the magnet sits on the 120T's
face and stands over the chip at home."""
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
    home_board_box,
    home_board_points,
    home_board_y,
    home_bracket_frame,
    home_chip_xz,
    home_outline,
    home_reliefs,
    home_screw_points,
    home_sensor_frame,
    home_to_base,
    inner_r,
)
from lib.base.params import YAW_BELT_Y
from lib.belts import GT2_BELT_W
from lib.fasteners import M2_SHCS, M3_PITCH, nut_volume, shcs_volume
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
POLE_Y = FACE_Y - HOME_MAGNET_T                                   # the magnet's south pole's face, glued on under it
CHIP_TOP = home_board_y() + KY003_BOARD_T + A3144_BODY_T          # the chip's branded face
HALL_Y = CHIP_TOP - A3144_HALL_DEPTH                              # its hall element
MARGIN = 2.5          # deg: the zone's far edge this far past the - end and short of the + end, for every operate point
PLUG_LEN = 14.0       # [ESTIMATE] a 3-pin Dupont housing on the header's pins, from the header's face
BOX_CLEAR = 0.5       # the board's footprint, its pins and the plug this far in from the plate's opening where it is there
# name -> (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCKS = {
    "yaw_home_bracket": (1, 2897.381, (-23.0, -55.0, 0.0), (43.0, 37.0, 3.0)),
    "yaw_home_magnet": (1, 84.823, (-3.0, -3.0, 0.0), (6.0, 6.0, 3.0)),
    "yaw_home_screws": (2, 269.849, (3.25, -52.25, -3.0), (14.5, 5.5, 13.0)),
    "yaw_home_nuts": (2, 91.818, (2.824574, -52.25, 5.6), (15.350853, 5.5, 2.4)),
    "yaw_home_board_screws": (2, 91.733, (-7.41, -33.9, -3.4), (3.8, 13.8, 10.0)),
    "yaw_home_board_nuts": (2, 34.287, (-7.51, -34.309401, -1.6), (4.0, 14.618802, 1.6)),
}
SENSOR = ("ky003_hall_sensor#1", "yaw_home_bracket#1", "yaw_home_screws#1", "yaw_home_nuts#1", "yaw_home_board_screws#1",
          "yaw_home_board_nuts#1")


def _deg(mm: float) -> float:
    """A length along the chip's circle as degrees of base_yaw."""
    return math.degrees(mm / H.chip_r)


def _in_opening(x: float, z: float, clear: float = BOX_CLEAR) -> bool:
    """Is (x, z) clear of the motor plate: inside its central opening, or in the sector where it runs out to the wall?"""
    deg = math.degrees(math.atan2(z, x)) % 360.0
    return PL.open_deg[0] <= deg <= PL.open_deg[1] or math.hypot(x, z) <= PL.opening_r - clear


def test_the_home_angle():
    """The magnet's spot on the chip's radius; on the 120T in its place it stands over the chip at BASE_YAW_HOME_DEG,
    inside the travel and near its - end; nothing is cut in the 120T for it; LEGACY has no sensor."""
    assert H.chip_r == YAW_MAGNET_R and LEGACY.home is None and not hasattr(YAW, "magnet")
    assert PARAMS.BASE_YAW_HOME_DEG == pytest.approx(yaw_magnet_deg() - H.chip_deg) == pytest.approx(-169.5)
    assert yaw_magnet_deg(deg=YAW_MAGNET_DEG + 10.0) == pytest.approx(yaw_magnet_deg() - 10.0)   # the pulley is flipped
    assert -PARAMS.BASE_YAW_LIMIT_DEG < PARAMS.BASE_YAW_HOME_DEG < -PARAMS.BASE_YAW_LIMIT_DEG + 10.0
    assert home_chip_xz() == pytest.approx((0.0, H.chip_r))


def test_the_switch_zone_fits_between_the_ends():
    """For both ends of the A3144's operate point: a strong field at the chip, and the switch's zone from past the - end
    to short of the + end (360 - 2 x the limit round the circle), MARGIN to spare both ways."""
    gap = POLE_Y - HALL_Y
    assert gap == pytest.approx(3.48, abs=0.01)
    assert disc_magnet_bz(HOME_MAGNET_DIA, HOME_MAGNET_T, 0.0, gap) >= 3.0 * max(A3144_OPERATE_MT)
    home, limit = PARAMS.BASE_YAW_HOME_DEG, PARAMS.BASE_YAW_LIMIT_DEG
    half = [_deg(switch_half_width(HOME_MAGNET_DIA, HOME_MAGNET_T, gap, mt)) for mt in A3144_OPERATE_MT]
    assert half == pytest.approx([12.8, 8.5], abs=0.1)              # the most sensitive chip's zone, the least's
    assert home - min(half) <= -limit - MARGIN                     # past the - end ...
    assert home - max(half) >= -(360.0 - limit) + MARGIN           # ... short of the + end, round the circle
    assert home + max(half) < 0.0                                  # the edge homing stops at: well inside the travel
    assert _deg(1.0) == pytest.approx(2.12, abs=0.01)              # the glued magnet 1 mm off its spot: home that far off


def test_the_layout():
    """The board on the bracket, level with the plate's underside; the chip 2.98 under the magnet's pole, the header 1.96
    under it and 4.96 under the 120T's face; the board, its pins and the plug clear of the plate and under the belt; the
    M3s under the plate - clear of its opening, its arc slot and the wall -, through the plate into their nuts, flush in
    pockets in its top, 4 pitches past them, with 2.6 of plate under the pockets; the M2s through the board's holes,
    past their nuts; the slots under the pins' (through the bracket) and the leads' tails."""
    assert home_board_y() == PL.y[0]
    assert POLE_Y - CHIP_TOP == pytest.approx(2.98) and FACE_Y - CHIP_TOP == pytest.approx(5.98)
    header_top = home_board_y() + KY003_BOARD_T + KY003_HEADER_H
    assert POLE_Y - header_top == pytest.approx(1.96) and FACE_Y - header_top == pytest.approx(4.96)
    (ox, oy, oz), _ = home_sensor_frame()
    hx, hy, _ = ky003_hall_point()
    assert (ox - hx, oz + hy) == pytest.approx(home_chip_xz()) and oy == home_board_y()
    reach = KY003_BOARD_L + KY003_PIN_OUT + PLUG_LEN
    for x, y in ((0.0, KY003_BOARD_W / 2.0), (0.0, -KY003_BOARD_W / 2.0), (KY003_BOARD_L, KY003_BOARD_W / 2.0),
                 (KY003_BOARD_L, -KY003_BOARD_W / 2.0), (reach, 1.5 * KY003_PIN_PITCH), (reach, -1.5 * KY003_PIN_PITCH)):
        assert _in_opening(*home_to_base(DEFAULT, x, y))
    plug_top = home_board_y() + KY003_BOARD_T + KY003_HEADER_H / 2.0 + KY003_PIN_PITCH / 2.0
    assert YAW_BELT_Y - GT2_BELT_W / 2.0 - plug_top >= 5.0
    for x, z in home_screw_points():
        r, deg = math.hypot(x, z), math.degrees(math.atan2(z, x))
        pocket = (H.nut.af + H.nut_clear) / math.sqrt(3.0)           # the pocket's corners off the screw's axis
        assert r - pocket >= PL.opening_r + 3.0 and PL.open_deg[1] - 360.0 < deg < PL.open_deg[0]
        assert abs(deg) - PL.arc_slot_half_deg >= 20.0
        assert z + pocket <= inner_r(DEFAULT, PL.y[1]) - 5.0
    tip = home_bracket_frame()[0][1] + H.screw_len
    assert tip - PL.y[1] == pytest.approx(2.0) and tip - PL.y[1] >= 4 * M3_PITCH
    assert PL.y[1] - PL.y[0] - H.nut.h == pytest.approx(2.6) and H.nut_clear > 0.0 and H.screw_hole_dia > H.screw.d
    assert H.board_screw_len - KY003_BOARD_T - H.bracket_t - H.board_nut.h == pytest.approx(1.8)
    assert H.board_hole_dia > M2_SHCS.d
    x0, x1, z0, z1 = home_board_box()
    assert all(x0 < x < x1 and z0 < z < z1 for x, z in home_board_points())
    (pin_x0, pin_x1, pin_z0, pin_z1, pin_d), (lead_x0, lead_x1, lead_z0, lead_z1, lead_d) = home_reliefs()
    assert pin_d == pytest.approx(KY003_PIN_TAIL + H.relief_past) and lead_d == pytest.approx(KY003_LEAD_TAIL + H.relief_past)
    assert pin_d > H.bracket_t > lead_d                              # the pins' slot through the bracket, the leads' blind
    assert pin_z1 - pin_z0 > 2.0 * KY003_PIN_PITCH and lead_z1 - lead_z0 > 2.0 * A3144_LEAD_PITCH
    assert x0 < pin_x0 < pin_x1 < lead_x0 < lead_x1 < x1
    xs, zs = zip(*home_outline(), strict=True)
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
    assert parts.load("yaw_home_nuts").MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * 2 * nut_volume(H.nut))
    assert parts.load("yaw_home_board_nuts").MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * 2 * nut_volume(H.board_nut))


@pytest.mark.slow
def test_the_slots_holes_and_pockets():
    """The bracket: the pins' slot open through it, the leads' slot to its depth, the M3 and M2 holes open; the base:
    each M3's hole open through the plate, its nut's hex pocket open from the plate's top down to its floor, solid
    under it and round it."""
    bracket, base = built.part("yaw_home_bracket"), built.part("base")
    t = H.bracket_t
    (px0, px1, pz0, pz1, _), (lx0, lx1, lz0, lz1, lead_d) = home_reliefs()
    assert not is_inside(bracket, (px0 + px1) / 2.0, -(pz0 + pz1) / 2.0, 0.2)
    x, z = (lx0 + lx1) / 2.0, (lz0 + lz1) / 2.0
    assert not is_inside(bracket, x, -z, t - lead_d + 0.2) and is_inside(bracket, x, -z, t - lead_d - 0.2)
    for x, z in home_board_points():
        assert not is_inside(bracket, x, -z, t / 2.0) and is_inside(bracket, x + H.board_hole_dia / 2.0 + 0.4, -z, t / 2.0)
    floor = PL.y[1] - H.nut.h
    for x, z in home_screw_points():
        assert not is_inside(bracket, x, -z, t / 2.0) and is_inside(bracket, x + H.screw_hole_dia / 2.0 + 0.4, -z, t / 2.0)
        assert not is_inside(base, x, PL.y[0] + 0.3, z) and not is_inside(base, x, floor - 0.3, z)   # the hole, through
        side = H.screw_hole_dia / 2.0 + 0.5                                                          # in the pocket, off the hole
        assert not is_inside(base, x + side, floor + 0.3, z) and is_inside(base, x + side, floor - 0.3, z)
        assert is_inside(base, x, floor + 0.3, z + (H.nut.af + H.nut_clear) / 2.0 + 0.4)            # round it


@pytest.mark.slow
def test_the_sensor_fits():
    """In the base's frame: the bracket on the plate's underside, the board on the bracket, the screws' heads on the
    board and under the bracket, the nuts under it and on their pockets' floors - touching, nothing overlapping; the
    KY-003 4.96 under the 120T's face, well off the belt; nothing of it near the 120T's screws."""
    into = "base#1"
    base = built.placed("base#1", into=into)
    ky, bracket, screws, nuts, board_screws, board_nuts = (built.placed(k, into=into) for k in SENSOR)
    magnet = built.placed("yaw_home_magnet#1", into=into)
    turning = [built.placed(k, into=into) for k in ("gt2_pulley_120t#1", "yaw_pulley_screws#1")]
    near = [base, built.placed("yaw_belt#1", into=into), built.placed("bearing_6806#2", into=into), *turning, magnet]
    kit = [ky, bracket, screws, nuts, board_screws, board_nuts]
    for part in kit:
        for other in near + kit:
            if other is not part:
                assert interference(part, other) < 0.01
    assert closest_points(bracket, base).distance < 0.01 and closest_points(ky, bracket).distance < 0.01
    assert closest_points(board_screws, ky).distance < 0.01 and closest_points(board_nuts, bracket).distance < 0.01
    assert closest_points(screws, bracket).distance < 0.01 and closest_points(nuts, screws).distance < 0.01
    assert closest_points(nuts, base).distance < 0.01
    assert closest_points(ky, turning[0]).distance == pytest.approx(4.96, abs=0.01)
    assert closest_points(ky, turning[1]).distance > 5.0 and closest_points(ky, near[1]).distance > 5.0


@pytest.mark.slow
def test_the_magnet_on_the_120t_over_the_chip_at_home():
    """In the capture frame W (base_yaw turns about its +Y): the magnet glued on the 120T's face - touching it, on its
    web, clear of its bolts' counterbores; at BASE_YAW_HOME_DEG its axis runs through the hall element, its south pole's
    face 3.48 over it; turning past the chip it keeps 2.5 or more off the sensor; at zero it is a quarter turn and more
    away."""
    pulley = built.placed("gt2_pulley_120t#1")
    magnet_shape = built.placed("yaw_home_magnet#1")
    assert interference(magnet_shape, pulley) < 1e-3 and closest_points(magnet_shape, pulley).distance < 0.01
    assert YAW_MAGNET_R + HOME_MAGNET_DIA / 2.0 <= YAW.rim_dia / 2.0 - 5.0
    a = math.radians(YAW_MAGNET_DEG)
    mx, mz = YAW_MAGNET_R * math.cos(a), YAW_MAGNET_R * math.sin(a)
    for bx, bz in YAW.bolt_points():
        assert math.hypot(mx - bx, mz - bz) - HOME_MAGNET_DIA / 2.0 - YAW.counterbore[0] / 2.0 >= 10.0
    hall = (P.location("ky003_hall_sensor#1", "world") * Location(ky003_hall_point())).position
    magnet = P.location("yaw_home_magnet#1", "world")
    assert magnet.position.Y == pytest.approx(POLE_Y) and hall.Y == pytest.approx(HALL_Y)
    home = Location((0.0, 0.0, 0.0), (0.0, PARAMS.BASE_YAW_HOME_DEG, 0.0)) * magnet
    axis = Axis(home.position, home.z_axis.direction)
    assert axis.direction.Y == pytest.approx(1.0)                        # up into the face: its pole face down
    assert math.hypot(home.position.X - hall.X, home.position.Z - hall.Z) < 1e-6
    assert home.position.Y - hall.Y == pytest.approx(3.48, abs=0.01)
    assert math.hypot(magnet.position.X - hall.X, magnet.position.Z - hall.Z) > H.chip_r
    sensor = [built.placed(k) for k in SENSOR[:2]] + [built.placed("yaw_home_board_screws#1")]
    for q in (PARAMS.BASE_YAW_HOME_DEG + d for d in (-5.5, -3.0, -1.0, 0.0, 1.0, 3.0, 6.0, 10.0)):
        turned = magnet_shape.moved(Location((0.0, 0.0, 0.0), (0.0, q, 0.0)))
        assert min(closest_points(turned, s).distance for s in sensor) >= 2.5
