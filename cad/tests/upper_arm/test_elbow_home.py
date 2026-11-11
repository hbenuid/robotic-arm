"""The elbow_pitch home sensor (lib/upper_arm/params.py ElbowHomeParams): a KY-003 hall module on a printed clip over the
upper arm's slab end, under the elbow 90T's outer face, and a Ø6 x 2 N52 magnet glued on that face, which stands over the
chip at ELBOW_PITCH_HOME_DEG (lib/params.py).

The home angle is where the switch's zone sits: it must run on past the lower end of the travel for every operate point
the A3144 may have (lib/sensors.py A3144_OPERATE_MT, switch_half_width()) - by more than the glued magnet can sit off its
mark (nothing on the 90T's face locates it) -, so the arm can never sit between the zone and that end; the edge homing
stops at lies well inside the travel. The clip, the magnet and the fasteners have no reference (lib/reference.py
NO_REFERENCE), so THIS file locks their numbers. The frames through the 90T's are the capture's (re-derived here from
placements.json); the clip's holes and slots are probed; the clip sits on the slab, the board on it; the magnet sits on
the 90T's face, stands over the chip at home and passes everything still over the travel, and the elbow's turning parts
pass the clip."""
import math

import pytest
from build123d import Location
from cad.build123d import closest_points

import parts
from assemblies._occurrences import placement_at, world_rows
from lib import params as PARAMS
from lib import placements as P
from lib import reference as R
from lib.datum import to_location
from lib.fasteners import M2_SHCS, nut_volume, shcs_volume
from lib.pulley.params import DEFAULT as PULLEY
from lib.sensors import (
    A3144_HALL_DEPTH,
    A3144_OPERATE_MT,
    KY003_BOARD_T,
    KY003_LEAD_TAIL,
    KY003_PIN_TAIL,
    disc_magnet_bz,
    ky003_hall_point,
    switch_half_width,
)
from lib.upper_arm import DEFAULT, LEGACY
from lib.upper_arm.layout import (
    home_board_points,
    home_chip_deg,
    home_chip_r,
    home_clip_frame,
    home_clip_z,
    home_end_x,
    home_hall_point,
    home_magnet_deg,
    home_magnet_face_y,
    home_magnet_frame,
    home_plate_box,
    home_reliefs,
    home_sensor_frame,
    home_slab_y,
    home_to_link,
)
from robot import frames as F
from tests import built
from tests.helpers import interference, is_inside

H = DEFAULT.home
GAP = H.air + A3144_HALL_DEPTH                     # the magnet's pole face .. the hall element, at home
MAG_R = home_chip_r()                              # the magnet's middle off the elbow axis: its zone's radius
GLUE_DEG = math.degrees(1.0 / MAG_R)               # [ESTIMATE] the glued magnet 1 mm off its mark round the axis
# name -> (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCKS = {
    "elbow_home_clip": (1, 4090.332, (-23.0, -22.02, 0.0), (26.2, 43.02, 21.5)),
    "elbow_home_magnet": (1, 56.549, (-3.0, -3.0, 0.0), (6.0, 6.0, 2.0)),
    "elbow_home_board_screws": (2, 79.167, (0.6, -6.9, -5.0), (3.8, 13.8, 8.0)),
    "elbow_home_board_nuts": (2, 34.287, (0.5, -7.309401, 1.6), (4.0, 14.618802, 1.6)),
}
SENSOR = ("ky003_hall_sensor#3", "elbow_home_board_screws#1", "elbow_home_board_nuts#1")
# elbow_link's parts that pass near the clip over the travel - the rest (the roll motor, its board and belt, the tower)
# stay above the roll frame's underside or far out along the forearm
NEAR = ("gt2_pulley_90t", "elbow_pulley_screws", "elbow_pulley_nuts", "forearm_roll_block", "elbow_flange",
        "forearm_roll_back_panel", "roll_back_magnets")
CLIP = "elbow_home_clip#1"


def _turned(shape, q: float):
    """A shape of elbow_link (at its world pose, the joint at zero) with elbow_pitch at q."""
    jw = F.joint_frame_world("elbow_pitch")
    return shape.moved(jw * Location((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), q) * jw.inverse())


def _elbow_link_parts(skip=("elbow_home_magnet#1",)) -> list:
    """(label, shape in W) of every part of elbow_link but `skip`, the joint at zero."""
    return [(f"{key}:{part}", built.part(part).moved(placement_at(part, world)))
            for key in F.LINKS["elbow_link"] if key not in skip for part, _role, world in world_rows(key)]


def _box_gap(a, b) -> float:
    d = (max(a.min.X - b.max.X, b.min.X - a.max.X, 0.0), max(a.min.Y - b.max.Y, b.min.Y - a.max.Y, 0.0),
         max(a.min.Z - b.max.Z, b.min.Z - a.max.Z, 0.0))
    return math.sqrt(sum(v * v for v in d))


def _nearest(movers, fixed, limit: float) -> tuple:
    """(distance, mover, fixed) of the nearest pair - exact under `limit`, else `limit` (their boxes that far apart)."""
    best = (limit, "", "")
    boxes = [(fl, fs, fs.bounding_box()) for fl, fs in fixed]
    for ml, ms in movers:
        mbb = ms.bounding_box()
        for fl, fs, fbb in boxes:
            if _box_gap(mbb, fbb) < best[0]:
                d = closest_points(ms, fs).distance
                if d < best[0]:
                    best = (d, ml, fl)
    return best


def test_the_home_angle():
    """The chip 22.2 deg round from the end flat toward -z, under the 90T's outer face; the magnet 17.66 deg round in the
    90T's own frame; home inside the travel, near its lower end; LEGACY has no sensor."""
    assert LEGACY.home is None and PARAMS.ELBOW_PITCH_HOME_DEG == H.home_deg == -41.0
    lower = PARAMS.ELBOW_PITCH_LIMITS_DEG[0]
    assert lower < PARAMS.ELBOW_PITCH_HOME_DEG < lower + 7.0
    assert MAG_R == pytest.approx(23.769729, abs=1e-6) and home_chip_deg() == pytest.approx(-22.249024, abs=1e-6)
    assert home_magnet_deg() == pytest.approx(17.662507, abs=1e-6)
    assert MAG_R + H.magnet_dia / 2.0 < PULLEY.flange_dia / 2.0 - 2.5          # on the outer face, inside its flange's edge
    assert MAG_R - H.magnet_dia / 2.0 > math.hypot(*PULLEY.bolt_points()[0]) + 7.4 / 2.0 + 2.0   # clear of the M4s' counterbores


def test_the_frames_through_the_90t_are_the_captures():
    """pulley_y / pulley_x_deg are the elbow 90T's frame in j1_link's, as placements.json records the two: its origin on
    the elbow axis at pulley_y, its +Y j1_link's -Y, its +X at pulley_x_deg; pulley_y is the belt's plane less half the
    tooth band."""
    t = P.location("j1_link#1", "world").inverse() * P.location("gt2_pulley_90t#3", "world")
    o, x, y = t.position, t.x_axis.direction, t.y_axis.direction
    assert (o.X, o.Y, o.Z) == pytest.approx((DEFAULT.slab.elbow_x, H.pulley_y, 0.0), abs=1e-4)
    assert (y.X, y.Y, y.Z) == pytest.approx((0.0, -1.0, 0.0), abs=1e-6)
    assert math.degrees(math.atan2(x.Z, x.X)) == pytest.approx(H.pulley_x_deg, abs=1e-5)
    assert H.pulley_y == pytest.approx(PARAMS.J1_BELT_Y + PULLEY.band_y1 / 2.0)
    assert home_magnet_face_y() == pytest.approx(-8.9)


def test_the_switch_zone_runs_past_the_lower_end():
    """For both ends of the A3144's operate point: a strong field at the chip, and the switch's zone past the lower end
    by more than the glued magnet can sit off its mark; the edge homing stops at well inside the travel."""
    assert GAP == pytest.approx(2.0)
    assert disc_magnet_bz(H.magnet_dia, H.magnet_t, 0.0, GAP) >= 5.0 * max(A3144_OPERATE_MT)
    half = [math.degrees(switch_half_width(H.magnet_dia, H.magnet_t, GAP, mt) / MAG_R) for mt in A3144_OPERATE_MT]
    assert half == pytest.approx([11.27, 9.06], abs=0.01)            # the most sensitive chip's zone, the least's
    home, (lower, _) = PARAMS.ELBOW_PITCH_HOME_DEG, PARAMS.ELBOW_PITCH_LIMITS_DEG
    assert GLUE_DEG == pytest.approx(2.41, abs=0.01)
    assert home - min(half) <= lower - GLUE_DEG                      # past the lower end, for every chip, glue and all
    assert home + max(half) <= lower + 18.0                          # the edge homing stops at


def test_the_layout():
    """The KY-003's branded face up N at the 90T's face, its hall element on home_hall_point(), `air` and its depth under
    the magnet's pole face; the clip's frame on the end flat at the slab's underside and its near side; the plate's
    near end on that side; the board's M2s through the plate and the board, into their nuts; the plate's slots under the
    pins' (through it) and the leads' tails."""
    assert home_to_link(DEFAULT, ky003_hall_point()) == pytest.approx(home_hall_point(), abs=1e-6)
    face = home_to_link(DEFAULT, (0.0, 0.0, 1.0))
    (ox, oy, oz), _ = home_sensor_frame()
    assert (face[0] - ox, face[1] - oy, face[2] - oz) == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)
    hx, hy, hz = home_hall_point()
    assert (hx, hz) == pytest.approx((DEFAULT.slab.elbow_x + H.chip_x, H.chip_z))
    assert home_magnet_face_y() - hy == pytest.approx(GAP)
    assert home_magnet_face_y() - (hy + A3144_HALL_DEPTH) == pytest.approx(H.air)   # the chip's branded face, `air` under
    assert home_end_x() == pytest.approx(206.0) and home_slab_y() == pytest.approx((5.5, 23.5))
    z0, z1 = home_clip_z()
    assert (z0, z1) == pytest.approx((-7.99, 8.01), abs=1e-6)
    assert home_clip_frame() == ((206.0, 5.5, pytest.approx(-7.99, abs=1e-9)), (0.0, 0.0, 0.0))
    px, py, pz = home_plate_box()
    assert pz == (-H.plate_t, 0.0)
    assert home_to_link(DEFAULT, (px[0], 0.0, 0.0))[2] == pytest.approx(z0)
    assert H.board_screw_len - H.plate_t - KY003_BOARD_T - H.board_nut.h == pytest.approx(-0.2)   # 1.4 of the nut's 1.6
    assert H.board_hole_dia > M2_SHCS.d
    assert all(px[0] < x < px[1] and py[0] < y < py[1] for x, y in home_board_points())
    (pin_x, _, pin_d), (lead_x, _, lead_d) = home_reliefs()
    assert pin_d == pytest.approx(KY003_PIN_TAIL + H.relief_past) and lead_d == pytest.approx(KY003_LEAD_TAIL + H.relief_past)
    assert pin_d > H.plate_t > lead_d
    assert px[0] < lead_x[0] < lead_x[1] < pin_x[0] < pin_x[1] < px[1]
    # the magnet on the 90T's outer face, its +Z in at it
    (mx, my, mz), rot = home_magnet_frame()
    assert my == pytest.approx(PULLEY.face_y + H.magnet_t) and rot == (90.0, 0.0, 0.0)
    assert math.hypot(mx, mz) == pytest.approx(MAG_R) and math.degrees(math.atan2(mz, mx)) == pytest.approx(home_magnet_deg())


def test_the_parts_have_no_reference():
    for name, (pieces, volume, lo, size) in LOCKS.items():
        mod, shape = parts.load(name), parts.build(name)
        assert parts.GROUPS[name] == "joints" and name not in parts.unplaced() and len(shape.solids()) == pieces
        assert parts.bought(name) == (name != "elbow_home_clip")
        assert R.NO_REFERENCE[name].startswith(("parts/joints/", "lib/upper_arm/home.py"))
        assert R.solid_volume(shape) == pytest.approx(volume, abs=1e-3)
        bb = shape.bounding_box()
        assert tuple(bb.min) == pytest.approx(lo, abs=1e-6) and tuple(bb.size) == pytest.approx(size, abs=1e-6)
        if parts.bought(name):
            assert mod.PURCHASE_QTY == pieces
    assert parts.load("elbow_home_magnet").MASS_G == pytest.approx(PARAMS.NDFEB_DENSITY * LOCKS["elbow_home_magnet"][1], abs=1e-3)
    assert parts.load("elbow_home_board_screws").MASS_G == pytest.approx(
        PARAMS.STEEL_DENSITY * 2 * shcs_volume(H.board_screw, H.board_screw_len))
    assert parts.load("elbow_home_board_nuts").MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * 2 * nut_volume(H.board_nut))


@pytest.mark.slow
def test_the_clip_holes_and_slots():
    """The clip: each M2 hole open through the plate and its teardrop's peak open over it toward +z (up as it prints),
    solid below it; the pins' slot open through the plate."""
    clip = built.part("elbow_home_clip")
    to_clip = to_location(home_clip_frame()).inverse()

    def at(p):    # a point of the KY-003's frame in the clip's
        v = (to_clip * Location(home_to_link(DEFAULT, p))).position
        return v.X, v.Y, v.Z

    mid = -H.plate_t / 2.0
    r = H.board_hole_dia / 2.0
    for x, y in home_board_points():
        assert not is_inside(clip, *at((x, y, mid)))
        assert not is_inside(clip, *at((x + r * 1.3, y, mid)))        # the peak, toward the KY-003's +x = +z
        assert is_inside(clip, *at((x - r * 1.3, y, mid)))            # solid below the hole
    (pin_x, _, _), _ = home_reliefs()
    assert not is_inside(clip, *at(((pin_x[0] + pin_x[1]) / 2.0, 0.0, mid)))
    # the face it prints on: every corner of the clip at z >= 0, its spine, arms and plate's near end on z = 0
    assert clip.bounding_box().min.Z == pytest.approx(0.0, abs=1e-9)


@pytest.mark.slow
def test_the_sensor_fits():
    """In j1_link's frame: the clip `gap` off the slab, nothing overlapping it; the board on the plate, the screws' heads
    under it and the nuts on the board - touching, nothing overlapping anything still; the clip and the board clear of
    the belt, its idler and the elbow motor's gearbox."""
    into = "j1_link#1"
    slab = built.placed(into, into=into)
    clip = built.placed(CLIP, into=into)
    ky, screws, nuts = (built.placed(k, into=into) for k in SENSOR)
    still = [slab] + [built.placed(k, into=into) for k in ("bearing_6806#3", "bearing_6806#4", "elbow_belt#1", "gt2_idler_20t#1",
                                                           "elbow_idler_spacer#1", "elbow_idler_screw#1", "eg17_g5_gearbox#1",
                                                           "gt2_pulley_20t_8mm#1")]
    kit = [clip, ky, screws, nuts]
    for part in kit:
        for other in still + kit:
            if other is not part:
                assert interference(part, other) < 0.01
    assert closest_points(clip, slab).distance == pytest.approx(H.gap, abs=1e-3)
    assert closest_points(ky, clip).distance < 0.01 and closest_points(nuts, ky).distance < 0.01
    assert closest_points(screws, clip).distance < 0.01
    for part in kit:
        assert min(closest_points(part, s).distance for s in still[1:]) >= 3.0


@pytest.mark.slow
def test_the_magnet_on_the_90t_over_the_chip_at_home():
    """The magnet glued on the 90T's outer face - touching it; at ELBOW_PITCH_HOME_DEG its axis runs through the hall
    element, its pole face `air` + the element's depth off it; over the travel it keeps 1.4 or more off the sensor (the
    outer nut, 1.42) and 4 off the clip; the elbow's turning parts keep 2.3 or more off the clip and the sensor (the
    90T's rim off the jaw), the roll frame 2.9 over the clip's top arm; at zero the magnet is far away."""
    pulley = built.placed("gt2_pulley_90t#3")
    magnet = built.placed("elbow_home_magnet#1")
    assert interference(magnet, pulley) < 1e-3 and closest_points(magnet, pulley).distance < 0.01
    hall = (P.location("ky003_hall_sensor#3", "world") * Location(ky003_hall_point())).position
    loc = P.location("elbow_home_magnet#1", "world")
    jw = F.joint_frame_world("elbow_pitch")
    home = jw * Location((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), PARAMS.ELBOW_PITCH_HOME_DEG) * jw.inverse() * loc
    v = hall - home.position
    along = -v.dot(home.z_axis.direction)
    # 2.5 um off the axis: the capture puts the 90T's axis ~3 um off the elbow joint's (placements.json), turned 41 deg
    assert along == pytest.approx(GAP, abs=1e-3) and (v + home.z_axis.direction * along).length < 5e-3
    assert (hall - loc.position).length > 15.0
    sensor = [(k, built.placed(k)) for k in SENSOR]
    clip = [(CLIP, built.placed(CLIP))]
    turning = [p for p in _elbow_link_parts() if p[0].rsplit(":", 1)[-1] in NEAR]
    assert len(turning) == len(NEAR)
    lower, upper = PARAMS.ELBOW_PITCH_LIMITS_DEG
    near_sensor = near_clip = near_turning = math.inf
    for q in [lower + 4.0 * i for i in range(int((upper - lower) / 4.0) + 1)] + [upper, PARAMS.ELBOW_PITCH_HOME_DEG]:
        m = [("magnet", _turned(magnet, q))]
        near_sensor = min(near_sensor, _nearest(m, sensor, 5.0)[0])
        near_clip = min(near_clip, _nearest(m, clip, 5.0)[0])
        d = _nearest([(label, _turned(s, q)) for label, s in turning], sensor + clip, 5.0)
        near_turning = min(near_turning, d[0])
    assert near_sensor == pytest.approx(1.42, abs=0.02)
    assert near_clip >= 4.0
    assert near_turning >= 2.3
    block = [p for p in turning if p[0].endswith(":forearm_roll_block")]
    assert len(block) == 1
    top = min(_nearest([("block", _turned(block[0][1], q))], clip, 5.0)[0] for q in range(int(lower), int(upper) + 1, 8))
    assert top >= 2.9
