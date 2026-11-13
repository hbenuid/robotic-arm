"""The forearm_roll home sensor (lib/forearm/params.py RollHomeParams): a KY-003 hall module on a printed carrier behind
the roll shaft, in the roll frame's bay under the back panel, and a Ø6 x 3 N52 magnet pressed flush into the shaft's
rear face, which stands over the chip at FOREARM_ROLL_HOME_DEG (lib/params.py). The carrier sits on the back panel's
opening's floor, its tube in the frame's wire hole (which runs on into the motor's pocket behind the board's cover),
the panel's rim in its notch.

The home angle is where the switch's zone sits: it must run on past the lower end of the travel for every operate point
the A3144 may have (lib/sensors.py A3144_OPERATE_MT, switch_half_width()) - by more than the carrier can sit off its
place -, so the forearm can never sit between the zone and that end; the edge homing stops at lies well inside the
travel. The carrier, the magnet and the fasteners have no reference (lib/reference.py NO_REFERENCE), so THIS file locks
their numbers. Everything here is in forearm_roll_drive's module frame (the joint's axis is its +Z)."""
import math

import pytest
from build123d import Location, Sphere, Vector
from cad.build123d import closest_points

import parts
from assemblies import forearm_roll_drive as M
from lib import params as PARAMS
from lib import reference as R
from lib.fasteners import M2_SHCS, nut_volume, shcs_volume
from lib.forearm import DEFAULT, motor_pocket, stack_positions
from lib.forearm.roll import (
    back_point,
    home_board_points,
    home_hall_point,
    home_magnet_frame,
    home_sensor_frame,
    home_strip_x,
    home_wire_axis,
)
from lib.forearm.roll_home import tube_end
from lib.motors import MKS_SERVO42D_STACK
from lib.sensors import (
    A3144_BODY_T,
    A3144_HALL_DEPTH,
    A3144_OPERATE_MT,
    KY003_BOARD_T,
    KY003_HEADER_H,
    disc_magnet_bz,
    ky003_hall_point,
    switch_half_width,
)
from tests import built
from tests.helpers import interference, is_inside

D, S = DEFAULT.drive, stack_positions(DEFAULT)
H, B = D.home, D.back
GAP = H.air + A3144_HALL_DEPTH                     # the magnet's pole face .. the hall element, at home
PLAY_DEG = math.degrees(0.3 / H.chip_r)            # the carrier 0.3 off its place (its play round the tube, the rim's notch)
# name -> (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCKS = {
    "roll_home_carrier": (1, 1971.487, (-13.555236, -50.725974, -28.349209), (11.0, 54.424662, 24.979209)),
    "roll_home_magnet": (1, 84.823, (-3.0, -3.0, 0.0), (6.0, 6.0, 3.0)),
    "roll_home_board_screws": (2, 79.167, (0.6, -6.9, -5.0), (3.8, 13.8, 8.0)),
    "roll_home_board_nuts": (2, 34.287, (0.5, -7.309401, 1.6), (4.0, 14.618802, 1.6)),
}
KIT = ("roll_home_carrier", "ky003_hall_sensor:forearm_roll", "roll_home_board_screws", "roll_home_board_nuts")
ROTOR = sorted(M.ROTOR)


@pytest.fixture(scope="module")
def module():
    return built.model(M.forearm_roll_drive)


def _leaf(module, label):
    return next(c for c in module.children if c.label == label)


def _turned(shape, q: float):
    """`shape` turned q degrees about the roll axis (the module's +Z): the rotor at forearm_roll q."""
    return shape.moved(Location((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), q))


def test_the_home_angle():
    """Home inside the travel near its lower end; the chip chip_r off the axis home_deg round from the magnet; the magnet
    in the shaft's flange in line with the stop lug (clear of the clamp's nuts: the slow test)."""
    assert PARAMS.FOREARM_ROLL_HOME_DEG == H.home_deg == -160.0
    lower = -PARAMS.FOREARM_ROLL_LIMIT_DEG
    assert lower < PARAMS.FOREARM_ROLL_HOME_DEG < lower + 7.0
    hx, hy, _hz = home_hall_point()
    assert math.hypot(hx, hy) == pytest.approx(H.chip_r) and math.degrees(math.atan2(hy, hx)) == pytest.approx(-160.0)
    (mx, my, mz), rot = home_magnet_frame()
    assert (mx, my, mz) == pytest.approx((H.chip_r, 0.0, S["z_shaft_end"])) and rot == (0.0, 0.0, 0.0)
    assert H.magnet_deg == 0.0                                                    # the stop lug's angle in the zero pose
    assert D.collar_od / 2.0 - (H.chip_r + H.magnet_dia / 2.0 + H.pocket_fit) == pytest.approx(1.45)   # the flange's wall


def test_the_switch_zone_runs_past_the_lower_end():
    """For both ends of the A3144's operate point: a strong field at the chip (the header holds the chip 2.0 off the
    shaft, so 4.8x the highest operate point, not the 5x of the clipped sensors at 1.5) and the switch's zone past the
    lower end by more than the carrier can sit off its place; the edge homing stops at inside the travel."""
    assert GAP == pytest.approx(2.5)
    assert disc_magnet_bz(H.magnet_dia, H.magnet_t, 0.0, GAP) >= 4.8 * max(A3144_OPERATE_MT)
    half = [math.degrees(switch_half_width(H.magnet_dia, H.magnet_t, GAP, mt) / H.chip_r) for mt in A3144_OPERATE_MT]
    assert half == pytest.approx([20.06, 15.24], abs=0.01)            # the most sensitive chip's zone, the least's
    home, lower = PARAMS.FOREARM_ROLL_HOME_DEG, -PARAMS.FOREARM_ROLL_LIMIT_DEG
    assert PLAY_DEG == pytest.approx(1.11, abs=0.01)
    assert home - min(half) <= lower - 8.0 - PLAY_DEG                 # 9.2 past the lower end or more, play and all
    assert home + max(half) == pytest.approx(-139.94, abs=0.01)       # the edge homing stops at (the most sensitive chip)


def test_the_layout():
    """The KY-003 unturned, its hall element on home_hall_point(), its branded face `air` behind the shaft's rear face and
    its header 0.98 behind it; the board's holes on the carrier's plate; the tube's axis from the opening's floor, flush
    with the strip's -X side, down and back to wire_end in the motor's pocket; the screws' heads under the plate, the nuts
    on the board."""
    (ox, oy, oz), rot = home_sensor_frame()
    hx, hy, hz = ky003_hall_point()
    assert rot == (0.0, 0.0, 0.0) and (ox + hx, oy + hy, oz + hz) == pytest.approx(home_hall_point())
    face = oz + KY003_BOARD_T + A3144_BODY_T
    assert S["z_shaft_end"] - face == pytest.approx(H.air)
    assert S["z_shaft_end"] - (oz + KY003_BOARD_T + KY003_HEADER_H) == pytest.approx(0.98)     # the tallest part
    assert all(H.plate_x[0] < x < H.plate_x[1] and abs(y) < H.plate_w / 2.0 for x, y in home_board_points())
    x0, x1 = home_strip_x()
    assert (x0, x1) == pytest.approx((ox + H.plate_x[0], ox + H.plate_x[1]))
    p, u, n = home_wire_axis()
    assert p == pytest.approx(back_point(x0 + H.tube_od / 2.0, H.tube_s, H.tube_depth))
    assert (p[1] + u[1] * n, p[2] + u[2] * n) == pytest.approx(H.wire_end)
    assert math.degrees(math.atan2(-u[2], -u[1])) == pytest.approx(28.53, abs=0.01)   # down and back from -Y
    y0, y1, z0, _z1 = motor_pocket()
    assert y0 < H.wire_end[0] < y1 and z0 < H.wire_end[1]                              # in the pocket's slide room ...
    mks_back = S["z_motor_board"] - MKS_SERVO42D_STACK
    assert H.wire_end[1] < mks_back                                                     # ... behind the board's cover
    assert tube_end() == pytest.approx((-10.555236, -49.293279, -25.713419), abs=1e-5)
    assert H.board_screw_len - H.plate_t - KY003_BOARD_T - H.board_nut.h == pytest.approx(-0.2)   # 1.4 of the nut's 1.6


def test_the_parts_have_no_reference():
    for name, (pieces, volume, lo, size) in LOCKS.items():
        mod, shape = parts.load(name), parts.build(name)
        assert parts.GROUPS[name] == "joints" and name not in parts.unplaced() and len(shape.solids()) == pieces
        assert parts.bought(name) == (name != "roll_home_carrier")
        assert R.NO_REFERENCE[name].startswith(("parts/joints/", "lib/forearm/roll_home.py"))
        assert R.solid_volume(shape) == pytest.approx(volume, abs=1e-3)
        bb = shape.bounding_box()
        assert tuple(bb.min) == pytest.approx(lo, abs=1e-6) and tuple(bb.size) == pytest.approx(size, abs=1e-6)
        if parts.bought(name):
            assert mod.PURCHASE_QTY == pieces
    assert parts.load("roll_home_magnet").MASS_G == pytest.approx(PARAMS.NDFEB_DENSITY * LOCKS["roll_home_magnet"][1], abs=1e-3)
    assert parts.load("roll_home_board_screws").MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * 2 * shcs_volume(H.board_screw, H.board_screw_len))
    assert parts.load("roll_home_board_nuts").MASS_G == pytest.approx(PARAMS.STEEL_DENSITY * 2 * nut_volume(H.board_nut))
    assert H.board_screw == M2_SHCS


@pytest.mark.slow
def test_the_carrier_holes_and_tube():
    """Each M2 hole open through the plate, its teardrop's peak open toward +X (up as it prints on its -X side) and solid
    toward -X; each head's pocket open under the plate; the tube's bore open along its axis to its end; the tube flush with
    the carrier's -X side (it prints along the bed)."""
    carrier = built.part("roll_home_carrier")
    sensor = Location(home_sensor_frame()[0])
    r = H.board_hole_dia / 2.0

    def at(x, y, z):
        v = (sensor * Location((x, y, z))).position
        return v.X, v.Y, v.Z

    mid = -H.plate_t / 2.0
    for x, y in home_board_points():
        assert not is_inside(carrier, *at(x, y, mid))
        assert not is_inside(carrier, *at(x + r * 1.3, y, mid))      # the peak, toward +X
        assert is_inside(carrier, *at(x - r * 1.3, y, mid))           # solid on the bed's side
        assert not is_inside(carrier, *at(x, y, -H.plate_t - 1.0))    # the head's pocket
    p, u, _n = home_wire_axis()
    for t in (0.0, 4.0, 8.0, H.tube_len - 0.5):
        assert not is_inside(carrier, p[0] + u[0] * t, p[1] + u[1] * t, p[2] + u[2] * t)
        q = (p[0] + u[0] * t - (H.tube_id + H.tube_od) / 4.0, p[1] + u[1] * t, p[2] + u[2] * t)
        assert is_inside(carrier, *q)                                 # the tube's wall
    assert carrier.bounding_box().min.X == pytest.approx(home_strip_x()[0], abs=1e-6)
    assert p[0] - H.tube_od / 2.0 == pytest.approx(home_strip_x()[0])


@pytest.mark.slow
def test_the_sensor_fits(module):
    """In the module: nothing of the sensor overlaps anything; the carrier on the opening's floor, the KY-003 on its
    plate, the screws' heads under it, the nuts on the board; the closed panel's rim key_gap off the carrier (its notch)
    and the KY-003 3.6 or more off the panel; the wire hole open from the opening's floor along the tube to the motor's
    pocket, the tube in it."""
    kit = {label: _leaf(module, label) for label in KIT}
    others = [c for c in module.children if c.label not in KIT and c.label != "roll_home_magnet"]
    frame = _leaf(module, "forearm_roll_block")
    for label, part in kit.items():
        for other in others + [s for k, s in kit.items() if k != label]:
            assert interference(part, other) < 0.05, f"{label} x {other.label}"
    carrier, ky = kit["roll_home_carrier"], kit["ky003_hall_sensor:forearm_roll"]
    assert closest_points(carrier, frame).distance < 0.01
    assert closest_points(ky, carrier).distance < 0.01
    assert closest_points(kit["roll_home_board_nuts"], ky).distance < 0.01
    assert closest_points(kit["roll_home_board_screws"], carrier).distance < 0.01
    panel = _leaf(module, "forearm_roll_back_panel")
    assert closest_points(carrier, panel).distance == pytest.approx(H.key_gap, abs=1e-3)
    assert closest_points(ky, panel).distance >= 3.6
    assert closest_points(ky, frame).distance >= 7.9
    p, u, n = home_wire_axis()
    for t in [n * i / 10.0 for i in range(11)] + [n + H.wire_past * 0.5]:
        assert not is_inside(frame, p[0] + u[0] * t, p[1] + u[1] * t, p[2] + u[2] * t), f"the wire hole shut at {t:.1f}"
    mks = _leaf(module, "mks_servo42d:forearm_roll")
    end = Vector(*H.wire_end)
    assert not is_inside(mks, end.X, end.Y, end.Z)
    assert closest_points(Location(tuple(end)) * Sphere(0.01), mks).distance > 1.0


@pytest.mark.slow
def test_the_magnet_in_the_shaft_over_the_chip_at_home(module):
    """The magnet in its pocket in the shaft's rear face - touching it, flush; at FOREARM_ROLL_HOME_DEG its axis runs
    through the hall element, its pole face `air` + the element's depth off it; over the travel the rotor keeps 0.98 or
    more off the sensor; at zero the magnet is far from the chip; it sits 3 or more off the clamp's nuts."""
    magnet, shaft = _leaf(module, "roll_home_magnet"), _leaf(module, "forearm_roll_shaft")
    assert interference(magnet, shaft) < 1e-3 and closest_points(magnet, shaft).distance < 0.01
    assert magnet.bounding_box().min.Z == pytest.approx(S["z_shaft_end"], abs=1e-6)
    hall = Vector(*home_hall_point())
    (mx, my, mz), _rot = home_magnet_frame()
    at_home = _turned(magnet, PARAMS.FOREARM_ROLL_HOME_DEG).bounding_box().center()
    assert (at_home.X, at_home.Y) == pytest.approx((hall.X, hall.Y), abs=1e-3)
    assert S["z_shaft_end"] - hall.Z == pytest.approx(GAP)
    assert (hall - Vector(mx, my, mz)).length > 25.0
    assert closest_points(magnet, _leaf(module, "roll_clamp_nuts")).distance > 3.0     # between two of the clamp's nuts
    rotor = [_leaf(module, label) for label in ROTOR]
    kit = [_leaf(module, label) for label in KIT]
    near = math.inf
    lim = PARAMS.FOREARM_ROLL_LIMIT_DEG
    for q in [-lim + 8.0 * i for i in range(int(2 * lim / 8.0) + 1)] + [lim, PARAMS.FOREARM_ROLL_HOME_DEG]:
        for part in rotor:
            moved = _turned(part, q)
            for k in kit:
                near = min(near, closest_points(moved, k).distance)
    assert near == pytest.approx(0.98, abs=0.02)
