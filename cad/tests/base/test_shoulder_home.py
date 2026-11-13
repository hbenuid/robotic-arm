"""The shoulder_pitch home sensor (lib/yaw_coupler/params.py HomeTrayParams): a KY-003 hall module on a printed tray
dropped into j1_coupler's pocket over the hub, under the cycloidal drive's turning shell, and a Ø6 x 3 N52 magnet glued
onto the outer face of one of the shell's pillars, which stands over the chip at SHOULDER_PITCH_HOME_DEG (lib/params.py).

The home angle is where the switch's zone sits: it must run on past the lower end of the travel for every operate point
the A3144 may have (lib/sensors.py A3144_OPERATE_MT, switch_half_width()) - by more than the glued magnet can sit off
its mark along the pillar's face -, so the arm can never sit between the zone and that end; the edge homing stops at
lies well inside the travel. The tray, the magnet and the fasteners have no reference (lib/reference.py NO_REFERENCE),
so THIS file locks their numbers. The layout and the tray's holes and slots are probed; the tray fits the pocket; the
magnet sits on its pillar, stands over the chip at home and passes everything still over the travel."""
import math

import pytest
from build123d import Location
from cad.build123d import closest_points

import parts
from lib import params as PARAMS
from lib import placements as P
from lib import reference as R
from lib.cycloidal import DEFAULT_CONFIG as DRIVE
from lib.fasteners import M2_SHCS, M4_SHCS, nut_volume, shcs_volume
from lib.sensors import (
    A3144_BODY_T,
    A3144_HALL_DEPTH,
    A3144_OPERATE_MT,
    HOME_MAGNET_DIA,
    HOME_MAGNET_T,
    KY003_BOARD_T,
    KY003_HEADER_H,
    KY003_LEAD_TAIL,
    KY003_PIN_TAIL,
    disc_magnet_bz,
    ky003_hall_point,
    switch_half_width,
)
from lib.yaw_coupler import DEFAULT, LEGACY, hole_points
from lib.yaw_coupler.layout import (
    home_alpha,
    home_block_points,
    home_board_points,
    home_hall_point,
    home_hall_r,
    home_pillar_deg,
    home_plate_box,
    home_reliefs,
    home_sensor_frame,
    home_to_coupler,
    home_tray_box,
    home_tray_frame,
)
from robot import frames as F
from tests import built
from tests.helpers import interference, is_inside

H, K, FK = DEFAULT.home, DEFAULT.yoke, DEFAULT.fork
GAP = H.air + A3144_HALL_DEPTH                     # the magnet's pole face .. the hall element, at home
MAG_R = DRIVE.housing.od / 2.0 + HOME_MAGNET_T / 2.0   # the magnet's middle off the drive's axis: its zone's radius
GLUE_DEG = math.degrees((8.8 - HOME_MAGNET_DIA) / 2.0 / MAG_R)   # [ESTIMATE] how far off its mark the magnet can sit: the
#                                                                pillar's face is ~8.8 wide at the od (its barrel edges chamfered)
TIP_TOP = K.pocket_y0 + 2.8                        # the 120T's M4s stand 2.8 out of the pocket's floor (HubParams' comment)
# name -> (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCKS = {
    "shoulder_home_tray": (1, 7072.626, (-14.8, -30.6, 0.0), (29.6, 61.2, 12.266878)),
    "shoulder_home_magnet": (1, 84.823, (-3.0, -3.0, 0.0), (6.0, 6.0, 3.0)),
    "shoulder_home_board_screws": (2, 79.167, (0.6, -6.9, -5.0), (3.8, 13.8, 8.0)),
    "shoulder_home_board_nuts": (2, 34.287, (0.5, -7.309401, 1.6), (4.0, 14.618802, 1.6)),
}
SENSOR = ("ky003_hall_sensor#2", "shoulder_home_tray#1", "shoulder_home_board_screws#1", "shoulder_home_board_nuts#1")


def _turned(shape, q: float):
    """A shape of upper_arm_link (at its world pose, the joint at zero) with shoulder_pitch at q."""
    jw = F.joint_frame_world("shoulder_pitch")
    return shape.moved(jw * Location((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), q) * jw.inverse())


def test_the_home_angle():
    """The magnet on pillar 2 of the shell; at SHOULDER_PITCH_HOME_DEG it stands 15.6 deg short of straight down toward
    the front, over the pocket; home inside the travel, near its lower end; LEGACY has no sensor."""
    assert LEGACY.home is None and PARAMS.SHOULDER_PITCH_HOME_DEG == H.home_deg == -118.0
    lower = PARAMS.SHOULDER_PITCH_LIMITS_DEG[0]
    assert lower < PARAMS.SHOULDER_PITCH_HOME_DEG < lower + 5.0
    assert home_pillar_deg() == pytest.approx(167.584167, abs=1e-6)
    assert home_alpha() == pytest.approx(164.415833, abs=1e-6)
    hx, hy, hz = home_hall_point()
    assert hx == pytest.approx(FK.face_x - H.magnet_z) == pytest.approx(0.0)
    assert math.hypot(hy - FK.axis_y, hz - FK.axis_z) == pytest.approx(home_hall_r())
    assert math.degrees(math.atan2(hz - FK.axis_z, hy - FK.axis_y)) % 360.0 == pytest.approx(home_alpha())


def test_the_switch_zone_runs_past_the_lower_end():
    """For both ends of the A3144's operate point: a strong field at the chip, and the switch's zone past the lower end
    by more than the glued magnet can sit off its mark; the edge homing stops at well inside the travel."""
    assert GAP == pytest.approx(2.0)
    assert disc_magnet_bz(HOME_MAGNET_DIA, HOME_MAGNET_T, 0.0, GAP) >= 6.0 * max(A3144_OPERATE_MT)
    half = [math.degrees(switch_half_width(HOME_MAGNET_DIA, HOME_MAGNET_T, GAP, mt) / MAG_R) for mt in A3144_OPERATE_MT]
    assert half == pytest.approx([4.37, 3.53], abs=0.01)             # the most sensitive chip's zone, the least's
    home, (lower, _) = PARAMS.SHOULDER_PITCH_HOME_DEG, PARAMS.SHOULDER_PITCH_LIMITS_DEG
    assert GLUE_DEG == pytest.approx(1.2, abs=0.05)
    assert home - min(half) <= lower - GLUE_DEG                      # past the lower end, for every chip, glue and all
    assert home + max(half) <= lower + 7.0                           # the edge homing stops at
    assert math.degrees(1.0 / MAG_R) == pytest.approx(0.87, abs=0.01)  # the glued magnet 1 mm off its mark: home that far off


def test_the_layout():
    """The KY-003's branded face toward the drive's axis, its hall element on home_hall_point(); the header 1.02 nearer
    the axis than the chip, still past the magnet's pole face; the tray the pocket's floor less wall_clear a side, holed
    round the M4s' tips, its finger slot at the back end; the block forward of the tips; the plate 0.5 or more over the
    tips and the tray where it hangs past the block; the board's M2s through the plate and the board, into their nuts;
    the plate's slots under the pins' (through it) and the leads' tails."""
    (_, oy, oz), _ = home_sensor_frame()
    hall = home_to_coupler(DEFAULT, ky003_hall_point())
    assert hall == pytest.approx(home_hall_point(), abs=1e-6)
    face = home_to_coupler(DEFAULT, (0.0, 0.0, 1.0))                 # +Z of the KY-003: toward the drive's axis
    a = math.radians(home_alpha())
    assert (face[1] - oy, face[2] - oz) == pytest.approx((-math.cos(a), -math.sin(a)), abs=1e-6)
    header_r = home_hall_r() - (KY003_HEADER_H - A3144_BODY_T) - A3144_HALL_DEPTH
    assert header_r - (DRIVE.housing.od / 2.0 + HOME_MAGNET_T) == pytest.approx(0.48, abs=1e-6)   # the header's top
    x0, x1, z0, z1 = home_tray_box()
    hx, hz = K.pocket_half
    assert (x0, x1, z0, z1) == pytest.approx((-hx + H.wall_clear, hx - H.wall_clear, -hz + H.wall_clear, hz - H.wall_clear))
    assert H.tray_t > TIP_TOP - K.pocket_y0
    assert home_tray_frame() == ((0.0, K.pocket_y0, 0.0), (-90.0, 0.0, 0.0))
    tip_z = max(abs(z) for _, z in hole_points()) + M4_SHCS.d / 2.0 + H.tip_clear
    assert H.block_z[0] - tip_z >= 2.5 and z1 - H.block_z[1] >= 0.5
    floor = [p for p in home_block_points() if p[1] == pytest.approx(K.pocket_y0 + H.tray_t)]
    assert len(floor) == 4
    px, py, pz = home_plate_box()
    far = [home_to_coupler(DEFAULT, (px[1], y, pz[0])) for y in py]  # the plate's far end, outer face
    assert min(p[1] for p in far) - (K.pocket_y0 + H.tray_t) >= 0.5
    assert H.board_screw_len - H.plate_t - KY003_BOARD_T - H.board_nut.h == pytest.approx(-0.2)   # 1.4 of the nut's 1.6
    assert H.board_hole_dia > M2_SHCS.d
    assert all(px[0] < x < px[1] and py[0] < y < py[1] for x, y in home_board_points())
    (pin_x, _, pin_d), (lead_x, _, lead_d) = home_reliefs()
    assert pin_d == pytest.approx(KY003_PIN_TAIL + H.relief_past) and lead_d == pytest.approx(KY003_LEAD_TAIL + H.relief_past)
    assert pin_d > H.plate_t > lead_d
    assert px[0] < lead_x[0] < lead_x[1] < pin_x[0] < pin_x[1] < px[1]


def test_the_blocks_foot_keeps_two_beads_off_the_head_recesses():
    """Where the block's slope meets the tray (block_z[0], across its width) it passes each board screw's head recess -
    tilted with the KY-003 - 0.8 or more off: two beads of a 0.4 nozzle, the printability check's minimum. The distance
    from that foot edge (along x) to the recess's axis, less its radius."""
    top, r = K.pocket_y0 + H.tray_t, H.board_screw.head_dia / 2.0 + H.head_recess
    for bx, by in home_board_points():
        p, q = home_to_coupler(DEFAULT, (bx, by, 0.0)), home_to_coupler(DEFAULT, (bx, by, -10.0))
        u = [qi - pi for qi, pi in zip(q, p, strict=True)]
        d = abs((top - p[1]) * u[2] - (H.block_z[0] - p[2]) * u[1]) / math.hypot(u[1], u[2])   # |(e - p) . (u x X)| / |u x X|
        assert d - r >= 0.8 - 1e-9, (bx, by, d - r)


def test_the_parts_have_no_reference():
    for name, (pieces, volume, lo, size) in LOCKS.items():
        mod, shape = parts.load(name), parts.build(name)
        assert parts.GROUPS[name] == "base" and name not in parts.unplaced() and len(shape.solids()) == pieces
        assert parts.bought(name) == (name != "shoulder_home_tray")
        assert R.NO_REFERENCE[name].startswith(("parts/base/", "lib/yaw_coupler/body.py"))
        assert R.solid_volume(shape) == pytest.approx(volume, abs=1e-3)
        bb = shape.bounding_box()
        assert tuple(bb.min) == pytest.approx(lo, abs=1e-6) and tuple(bb.size) == pytest.approx(size, abs=1e-6)
        if parts.bought(name):
            assert mod.PURCHASE_QTY == pieces
    assert parts.load("shoulder_home_magnet").MASS_G == pytest.approx(PARAMS.NDFEB_DENSITY * LOCKS["shoulder_home_magnet"][1],
                                                                      abs=1e-3)
    assert parts.load("shoulder_home_board_screws").MASS_G == pytest.approx(
        PARAMS.STEEL_DENSITY * 2 * shcs_volume(H.board_screw, H.board_screw_len))
    assert parts.load("shoulder_home_board_nuts").MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * 2 * nut_volume(H.board_nut))


@pytest.mark.slow
def test_the_tray_holes_and_slots():
    """The tray: open round each M4 tip, the finger slot open through it, solid between; the M2 holes open through the
    plate; the pins' slot open through the plate."""
    tray = built.part("shoulder_home_tray")
    t = H.tray_t

    def at(p):    # a point of the coupler's frame in the tray's: (x, -z, y - pocket_y0)
        return (p[0], -p[2], p[1] - K.pocket_y0)

    for x, z in hole_points():
        assert not is_inside(tray, x, -z, t / 2.0) and is_inside(tray, x + 3.6, -z, t / 2.0)
    _, _, z0, _ = home_tray_box()
    assert not is_inside(tray, 0.0, -(z0 + H.slot[0] / 2.0), t / 2.0) and is_inside(tray, 0.0, -(z0 + H.slot[0] + 1.0), t / 2.0)
    for x, y in home_board_points():
        assert not is_inside(tray, *at(home_to_coupler(DEFAULT, (x, y, -H.plate_t / 2.0))))
        assert is_inside(tray, *at(home_to_coupler(DEFAULT, (x + 2.0, y, -H.plate_t / 2.0))))
    (pin_x, _, _), _ = home_reliefs()
    assert not is_inside(tray, *at(home_to_coupler(DEFAULT, ((pin_x[0] + pin_x[1]) / 2.0, 0.0, -H.plate_t / 2.0))))


@pytest.mark.slow
def test_the_sensor_fits():
    """In the coupler's frame: the tray on the pocket's floor, wall_clear off its walls, clear of the M4s; the board on
    the plate, the screws' heads under it and the nuts on the board - touching, nothing overlapping anything still."""
    into = "j1_coupler#1"
    coupler = built.placed(into, into=into)
    ky, tray, screws, nuts = (built.placed(k, into=into) for k in SENSOR)
    still = [coupler, built.placed("yaw_pulley_screws#1", into=into), built.placed("yaw_pulley_nuts#1", into=into),
             built.placed("j1_motor_leg#1", into=into)]
    kit = [ky, tray, screws, nuts]
    for part in kit:
        for other in still + kit:
            if other is not part:
                assert interference(part, other) < 0.01
    assert closest_points(tray, coupler).distance < 0.01
    assert closest_points(ky, tray).distance < 0.01 and closest_points(nuts, ky).distance < 0.01
    assert closest_points(screws, tray).distance < 0.01
    assert closest_points(tray, still[1]).distance >= H.tip_clear - 0.01


@pytest.mark.slow
def test_the_magnet_on_its_pillar_over_the_chip_at_home():
    """The magnet glued on the pillar's face - touching the drive's shell (j1_link), clear of the housing bolts; at
    SHOULDER_PITCH_HOME_DEG its axis runs through the hall element, its pole face `air` + the element's depth off it;
    over the travel it keeps 6 or more off the coupler and 1.4 or more off the sensor, and the shell's plain pillars keep
    4 or more off the sensor; at zero it is far away."""
    shell = built.placed("j1_link#1")
    magnet = built.placed("shoulder_home_magnet#1")
    assert interference(magnet, shell) < 1e-3 and closest_points(magnet, shell).distance < 0.01
    hall = (P.location("ky003_hall_sensor#2", "world") * Location(ky003_hall_point())).position
    loc = P.location("shoulder_home_magnet#1", "world")
    jw = F.joint_frame_world("shoulder_pitch")
    home = jw * Location((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), PARAMS.SHOULDER_PITCH_HOME_DEG) * jw.inverse() * loc
    v = hall - home.position
    along = -v.dot(home.z_axis.direction)
    assert along == pytest.approx(GAP, abs=1e-3) and (v + home.z_axis.direction * along).length < 1e-3
    assert (hall - loc.position).length > 50.0
    still = [built.placed(k) for k in ("j1_coupler#1", *SENSOR)]
    sensor = still[1:]
    lower, upper = PARAMS.SHOULDER_PITCH_LIMITS_DEG
    for q in [lower + 4.0 * i for i in range(int((upper - lower) / 4.0) + 1)] + [upper, PARAMS.SHOULDER_PITCH_HOME_DEG]:
        m, s = _turned(magnet, q), _turned(shell, q)
        assert closest_points(m, still[0]).distance >= 6.0
        assert min(closest_points(m, k).distance for k in sensor) >= 1.4
        assert min(closest_points(s, k).distance for k in sensor) >= 4.0
