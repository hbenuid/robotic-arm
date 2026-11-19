"""The wrist_roll home sensor (lib/wrist/params.py WristRollHomeParams): a KY-003 hall module upright on a printed fin
beside the roll pulley's face, on the far face's side, and a Ø6 x 2 N52 magnet glued on the face's rim, which stands over
the chip at WRIST_ROLL_HOME_DEG (lib/params.py). The fin is glued on the frame's front facet, front face and bearing boss
(0.2 off each): nothing on the arm changes.

The home angle is where the switch's zone sits: it must run on past the lower end of the travel for every operate point
the A3144 may have (lib/sensors.py A3144_OPERATE_MT, switch_half_width()) - by more than a magnet glued 1 mm off its
mark turns it -, so the gripper can never sit between the zone and that end; the edge homing stops at lies well inside
the travel. The fin, the magnet and the fasteners have no reference (lib/reference.py NO_REFERENCE), so THIS file locks
their numbers. Everything here is in wrist_roll_drive's module frame (the joint's axis is its +Z)."""
import math

import pytest
from build123d import Location, Vector
from cad.build123d import closest_points

import parts
from assemblies import wrist_roll_drive as M
from lib import params as PARAMS
from lib import reference as R
from lib.datum import to_location
from lib.fasteners import M2_NUT, M2_SHCS, nut_volume, shcs_volume
from lib.sensors import (
    A3144_BODY_T,
    A3144_HALL_DEPTH,
    A3144_OPERATE_MT,
    KY003_BOARD_T,
    disc_magnet_bz,
    ky003_hall_point,
    switch_half_width,
)
from lib.wrist import DEFAULT
from lib.wrist.layout import (
    connector_bolt_points_module,
    home_board_points,
    home_face_z,
    home_fin_back,
    home_fin_t,
    home_hall_point,
    home_magnet_deg,
    home_magnet_frame,
    home_reliefs,
    home_sensor_frame,
    module_frame_in_host,
    stack_positions,
)
from tests import built
from tests.helpers import Reach, interference, is_inside, nearest_under

D, S = DEFAULT.roll, stack_positions(DEFAULT)
H = D.home
FACE_R = D.face_dia / 2.0
GAP = H.air + A3144_HALL_DEPTH                     # the magnet's pole face .. the hall element, at home
R_HALL = home_hall_point()[0]                      # the hall element off the roll axis
GLUE_DEG = math.degrees(1.0 / (FACE_R + H.magnet_t / 2.0))   # the magnet glued 1 mm off its mark round the rim
# name -> (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCKS = {
    "wrist_roll_home_fin": (1, 2750.715, (21.5, -2.99, 8.782843), (8.5, 24.5, 31.867157)),
    "wrist_roll_home_magnet": (1, 56.549, (-3.0, -3.0, 0.0), (6.0, 6.0, 2.0)),
    "wrist_roll_home_board_screws": (2, 66.6, (0.6, -6.9, -2.4), (3.8, 13.8, 6.0)),
    "wrist_roll_home_board_nuts": (2, 34.287, (0.5, -7.309401, -2.28), (4.0, 14.618802, 1.6)),
}
KIT = ("wrist_roll_home_fin", "ky003_hall_sensor:wrist_roll", "wrist_roll_home_board_screws", "wrist_roll_home_board_nuts")
ROTOR = sorted(M.ROTOR)


@pytest.fixture(scope="module")
def module():
    return built.model(M.wrist_roll_drive)


@pytest.fixture(scope="module")
def frame():
    """wrist_link (the drive's frame, its host) in the module frame."""
    return built.part("wrist_link").moved(to_location(module_frame_in_host(DEFAULT)).inverse())


def _leaf(module, label):
    return next(c for c in module.children if c.label == label)


def _turned(shape, q: float):
    """`shape` turned q degrees about the roll axis (the module's +Z): the rotor at wrist_roll q."""
    return shape.moved(Location((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), q))


def test_the_home_angle():
    """Home inside the travel near its lower end; the chip on +X at the face's middle; the magnet on the face's rim
    -home_deg round from +X in the zero pose, 0.69 clear of the nearest of the connector's nut slots at the rim."""
    assert PARAMS.WRIST_ROLL_HOME_DEG == H.home_deg == -165.0
    lower = -PARAMS.WRIST_ROLL_LIMIT_DEG
    assert lower < PARAMS.WRIST_ROLL_HOME_DEG < lower + 2.0
    assert home_hall_point() == pytest.approx((FACE_R + H.magnet_t + H.air + A3144_HALL_DEPTH, 0.0, home_face_z()))
    assert home_face_z() == pytest.approx((S["z_ring_end"] + S["z_face"]) / 2.0)
    assert home_magnet_deg() == 165.0
    (mx, my, mz), _rot = home_magnet_frame()
    r = FACE_R + H.magnet_t
    assert (mx, my, mz) == pytest.approx((r * math.cos(math.radians(165.0)), r * math.sin(math.radians(165.0)), home_face_z()))
    assert H.magnet_dia == D.face_t                                    # it spans the face, edge to edge
    slot_half, mag_half = (D.nut.af + 2.0 * D.nut_fit) / 2.0, H.magnet_dia / 2.0
    clear = []
    for x, y in connector_bolt_points_module():
        apart = abs((math.degrees(math.atan2(y, x)) - home_magnet_deg() + 180.0) % 360.0 - 180.0)
        edges = math.degrees(math.asin(slot_half / FACE_R)) + math.degrees(math.asin(mag_half / FACE_R))
        clear.append(math.radians(apart - edges) * FACE_R)
    assert min(clear) == pytest.approx(0.69, abs=0.01)


def test_the_switch_zone_runs_past_the_lower_end():
    """For both ends of the A3144's operate point: a strong field at the chip (5x the highest operate point, as the
    clipped sensors) and the switch's zone past the lower end by more than a magnet 1 mm off its mark turns it; the edge
    homing stops at inside the travel."""
    assert GAP == pytest.approx(2.0)
    assert disc_magnet_bz(H.magnet_dia, H.magnet_t, 0.0, GAP) >= 5.0 * max(A3144_OPERATE_MT)
    half = [math.degrees(switch_half_width(H.magnet_dia, H.magnet_t, GAP, mt) / R_HALL) for mt in A3144_OPERATE_MT]
    assert half == pytest.approx([10.71, 8.62], abs=0.01)            # the most sensitive chip's zone, the least's
    home, lower = PARAMS.WRIST_ROLL_HOME_DEG, -PARAMS.WRIST_ROLL_LIMIT_DEG
    assert GLUE_DEG == pytest.approx(2.60, abs=0.01)
    assert home - min(half) <= lower - 5.0 - GLUE_DEG                 # 7.6 past the lower end, 5.0 with the magnet 1 mm off
    assert home + max(half) == pytest.approx(-154.29, abs=0.01)      # the edge homing stops at (the most sensitive chip)


def test_the_layout():
    """The KY-003 upright, its branded face toward the roll axis `air` off the magnet's pole face at home, its header
    toward the motor (+Y); the board's back on the fin, the fin's back flush with the far face; the board's holes inside
    the fin's plate; the screws through the board and the fin to its back, the nuts' full height on them in the pockets."""
    (ox, oy, oz), rot = home_sensor_frame()
    loc = to_location(((ox, oy, oz), rot))
    axes = [tuple(round(c, 9) + 0.0 for c in (loc * Location(v)).position - loc.position)
            for v in ((1, 0, 0), (0, 1, 0), (0, 0, 1))]
    assert axes == [(0.0, 1.0, 0.0), (0.0, 0.0, -1.0), (-1.0, 0.0, 0.0)]
    hall = (loc * Location(ky003_hall_point())).position
    assert tuple(hall) == pytest.approx(home_hall_point())
    face = ox - KY003_BOARD_T - A3144_BODY_T                            # the branded face (module x)
    assert face - (FACE_R + H.magnet_t) == pytest.approx(H.air)
    assert home_fin_back() == D.frame_r
    assert home_fin_t() == pytest.approx(2.38)
    assert all(H.plate_x[0] < x < H.plate_x[1] and abs(y) < H.plate_w / 2.0 for x, y in home_board_points())
    assert H.board_screw_len - KY003_BOARD_T - home_fin_t() == pytest.approx(0.02)   # the tip at the fin's back
    assert H.nut_pocket_depth - H.board_nut.h == pytest.approx(0.1) and H.board_nut == M2_NUT
    assert [d for _x, _y, d in home_reliefs()] == pytest.approx([3.5, 1.5])          # the pins' through, the leads' blind
    assert home_reliefs()[0][2] > home_fin_t() > home_reliefs()[1][2]


def test_the_parts_have_no_reference():
    for name, (pieces, volume, lo, size) in LOCKS.items():
        mod, shape = parts.load(name), parts.build(name)
        assert parts.GROUPS[name] == "wrist" and name not in parts.unplaced() and len(shape.solids()) == pieces
        assert parts.bought(name) == (name != "wrist_roll_home_fin")
        assert R.NO_REFERENCE[name].startswith(("parts/wrist/", "lib/wrist/home.py"))
        assert R.solid_volume(shape) == pytest.approx(volume, abs=1e-3)
        bb = shape.bounding_box()
        assert tuple(bb.min) == pytest.approx(lo, abs=1e-6) and tuple(bb.size) == pytest.approx(size, abs=1e-6)
        if parts.bought(name):
            assert mod.PURCHASE_QTY == pieces
    assert parts.load("wrist_roll_home_magnet").MASS_G == pytest.approx(PARAMS.NDFEB_DENSITY * LOCKS["wrist_roll_home_magnet"][1], abs=1e-3)
    assert parts.load("wrist_roll_home_board_screws").MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * 2 * shcs_volume(H.board_screw, H.board_screw_len))
    assert parts.load("wrist_roll_home_board_nuts").MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * 2 * nut_volume(H.board_nut))
    assert H.board_screw == M2_SHCS


@pytest.mark.slow
def test_the_fin_holes_and_pockets():
    """Each M2 hole open through the fin, its nut's hex pocket open from the back and solid past the hole beside it; the
    header pins' slot through, the leads' blind; the fin's back flush with the far face, nothing past it."""
    fin = built.part("wrist_roll_home_fin")
    loc = to_location(home_sensor_frame())
    t = home_fin_t()

    def at(x, y, z):
        v = (loc * Location((x, y, z))).position
        return v.X, v.Y, v.Z

    r = H.board_hole_dia / 2.0
    for x, y in home_board_points():
        assert not is_inside(fin, *at(x, y, -t / 2.0))                       # the hole, mid-fin
        assert is_inside(fin, *at(x + r + 0.3, y, -t / 2.0 + 0.6))            # its wall, over the pocket's floor
        assert not is_inside(fin, *at(x + r + 0.4, y, -t + 0.5))               # the pocket, wider than the hole
        assert is_inside(fin, *at(x + H.nut_pocket_af / 2.0 + 0.4, y, -t + 0.5))   # the fin past the pocket's flat
    (px, py, _depth), (lx, ly, ldepth) = [((x0 + x1) / 2.0, 0.0, d) for (x0, x1), _y, d in home_reliefs()]
    assert not is_inside(fin, *at(px, py, -t + 0.2))                        # the pins' slot open to the back
    assert not is_inside(fin, *at(lx, ly, -ldepth + 0.2)) and is_inside(fin, *at(lx, ly, -ldepth - 0.3))
    assert fin.bounding_box().max.X == pytest.approx(D.frame_r, abs=1e-6)


@pytest.mark.slow
def test_the_sensor_fits(module, frame):
    """In the module: nothing of the sensor overlaps anything (the frame either); the fin `gap` off the frame (the
    glue's), the KY-003 on the fin, the screws' heads on the board, the nuts in the fin; nothing past the far face."""
    kit = {label: _leaf(module, label) for label in KIT}
    others = [c for c in module.children if c.label not in KIT]
    for label, part in kit.items():
        for other in others + [frame] + [s for k, s in kit.items() if k != label]:
            assert interference(part, other) < 0.05, f"{label} x {other.label}"
    fin, ky = kit["wrist_roll_home_fin"], kit["ky003_hall_sensor:wrist_roll"]
    assert closest_points(fin, frame).distance == pytest.approx(H.gap, abs=1e-3)
    assert closest_points(ky, fin).distance < 0.01
    assert closest_points(kit["wrist_roll_home_board_screws"], ky).distance < 0.01
    assert closest_points(kit["wrist_roll_home_board_nuts"], fin).distance < 0.01
    assert closest_points(ky, frame).distance >= 5.9
    assert max(p.bounding_box().max.X for label, p in kit.items() if label not in ("ky003_hall_sensor:wrist_roll",
               "wrist_roll_home_board_screws")) <= D.frame_r + 1e-6
    assert kit["wrist_roll_home_board_screws"].bounding_box().max.X - D.frame_r == pytest.approx(0.02, abs=1e-6)   # the tips


@pytest.mark.slow
def test_the_magnet_on_the_rim_over_the_chip_at_home(module):
    """The magnet on the pulley face's rim - touching it, no overlap, within the face's thickness; at WRIST_ROLL_HOME_DEG
    its axis runs through the hall element, its pole face `air` + the element's depth off it; at zero the magnet is far
    from the chip; over the travel the rotor keeps 1.3 or more off the sensor (the disc's corners swing 0.2 nearer than
    its middle)."""
    magnet, pulley = _leaf(module, "wrist_roll_home_magnet"), _leaf(module, "wrist_roll_pulley")
    assert interference(magnet, pulley) < 1e-3 and closest_points(magnet, pulley).distance < 0.01
    bb = magnet.bounding_box()
    assert (bb.min.Z, bb.max.Z) == pytest.approx((S["z_ring_end"], S["z_face"]), abs=1e-6)
    hall = Vector(*home_hall_point())
    at_home = _turned(magnet, PARAMS.WRIST_ROLL_HOME_DEG).bounding_box()
    assert (at_home.center().Y, at_home.center().Z) == pytest.approx((hall.Y, hall.Z), abs=1e-3)
    assert hall.X - at_home.max.X == pytest.approx(GAP, abs=1e-3)
    (mx, my, mz), _rot = home_magnet_frame()
    assert (hall - Vector(mx, my, mz)).length > 40.0
    # the least distance over the travel, exact under 5 mm (helpers.nearest_under), else 5
    rotor = [Reach(_leaf(module, label)) for label in ROTOR]
    kit = [Reach(_leaf(module, label)) for label in KIT]
    near = 5.0
    lim = PARAMS.WRIST_ROLL_LIMIT_DEG
    for q in [-lim + 4.0 * i for i in range(int(2 * lim / 4.0) + 1)] + [lim, PARAMS.WRIST_ROLL_HOME_DEG, -158.0]:
        near = nearest_under(rotor, kit, near, Location((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), q))
    assert near == pytest.approx(1.31, abs=0.02)
