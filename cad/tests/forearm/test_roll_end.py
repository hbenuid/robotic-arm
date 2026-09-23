"""The DEFAULT forearm - the roll end: the flange wall replaces the elbow disc, the wrist-pitch motor sits where
the stock belt puts it and clears the wall, the caps end at the wall, the wrist end is the SolidWorks one."""
import math

import pytest
from build123d import Box, Pos

from lib import params as PARAMS
from lib import placements as P
from lib.belts import GT2_PULLEY_20T_TEETH, GT2_PULLEY_90T_TEETH, closed_belt_length
from lib.forearm import DEFAULT, LEGACY, cap1_socket_points, cap2_socket_points, flange_bolt_points, link_socket_points, motor_window
from lib.motors import NEMA17_40_BODY_W, NEMA17_40_CONNECTOR_D
from tests.forearm.helpers import in_host, interference, is_inside

import parts

R = DEFAULT.roll_end


def test_default_layout():
    assert DEFAULT.roll and not LEGACY.roll
    assert DEFAULT.web == LEGACY.web and DEFAULT.boss == LEGACY.boss and DEFAULT.sockets == LEGACY.sockets
    # the slide: set by the belt, inside the (shortened) slots, the plug clear of the wall, the body clear of the boss
    x = DEFAULT.motor_x
    assert math.isclose(closed_belt_length(x - DEFAULT.web.wrist_x, GT2_PULLEY_90T_TEETH, GT2_PULLEY_20T_TEETH), R.wrist_belt, abs_tol=1e-6)
    lo, hi = DEFAULT.slide_range
    assert lo < x < hi
    assert x + NEMA17_40_BODY_W / 2.0 + NEMA17_40_CONNECTOR_D + R.plug_clearance <= R.wall_x[0]
    assert x - NEMA17_40_BODY_W / 2.0 >= DEFAULT.web.wrist_x + DEFAULT.boss.dia / 2.0
    # the slots stop >= 4 mm before the wall and their arc centres bracket the slide range (the pilot boss in the
    # central slot, the bolt pattern's +/- 15.5 in the side slots)
    s = DEFAULT.slot
    assert s.centre_w >= 22.3 and s.centre_x[1] + s.centre_w / 2.0 <= R.wall_x[0] - 4.0
    assert s.side_x[0] <= lo - 15.5 and s.side_x[1] >= hi + 15.5
    assert s.centre_x[0] <= lo and s.centre_x[1] >= hi
    # the window: past the body toward the wrist, past the connector toward the elbow, inside the pocket (r > 55)
    x0, x1, half_w = motor_window(DEFAULT)
    assert x0 <= x - NEMA17_40_BODY_W / 2.0 - 2.0 and x1 >= x + NEMA17_40_BODY_W / 2.0 + NEMA17_40_CONNECTOR_D + 2.0
    assert math.hypot(x0 - DEFAULT.web.wrist_x, half_w) > DEFAULT.cap1.pocket_wrist_r
    # sockets: the columns the wall took stay out
    assert link_socket_points(DEFAULT) == ([(-120.0, 40.0), (-120.0, -40.0), (-165.0, 40.0), (-165.0, -40.0)],
                                           [(-120.0, 40.0), (-120.0, -40.0), (-165.0, 40.0), (-165.0, -40.0), (-244.64, 20.0), (-244.64, -20.0)])
    assert cap1_socket_points(DEFAULT) == link_socket_points(DEFAULT)[0] and cap2_socket_points(DEFAULT) == link_socket_points(DEFAULT)[1]
    assert len(flange_bolt_points(DEFAULT)) == 4 and (23.0, 25.0) in flange_bolt_points(DEFAULT)
    assert PARAMS.J2_MOTOR_SLIDE_X == x and PARAMS.FOREARM_ROLL_AXIS_Z == R.axis_z


@pytest.fixture(scope="module")
def link():
    return parts.build("j2_link")


@pytest.mark.slow
def test_link_ends_at_the_wall_and_keeps_its_wrist_end(link):
    from lib.forearm.link import build_link

    assert link.is_valid and len(link.solids()) == 1
    bb = link.bounding_box()
    assert math.isclose(bb.max.X, R.wall_x[1], abs_tol=1e-6) and math.isclose(bb.min.Z, R.wall_z[0], abs_tol=1e-6)
    assert math.isclose(bb.max.Z, R.wall_z[1], abs_tol=1e-6)
    # the wrist end (x < -165: the boss, the -165 sockets, the wrist pair) is the SolidWorks one
    probe = Pos(-210.0, 0.0, 15.0) * Box(90.0, 100.0, 60.0)
    assert abs(interference(link, probe) - interference(build_link(LEGACY), probe)) < 0.5
    # the wall: solid, the flange recess on its elbow face, the cable bore and the bolt holes through it
    assert is_inside(link, -92, 40, 50) and is_inside(link, -92, 0, -8)
    assert not is_inside(link, -89, 0, 25 + 29) and is_inside(link, -91, 0, 25 + 29)      # recess 2 deep, Ø60.3
    assert not is_inside(link, -92, 0, 25) and not is_inside(link, -92, 0, 25 + 13)      # cable bore Ø28
    assert is_inside(link, -92, 0, 25 + 15.5)
    for y, z in flange_bolt_points(DEFAULT):
        assert not is_inside(link, -92, y, z) and not is_inside(link, -95.9, y, z)
    # the slots: the central one passes the pilot, both stop before the wall
    assert not is_inside(link, DEFAULT.motor_x, 11.0, 13.5) and is_inside(link, DEFAULT.motor_x, 11.3, 13.5)
    assert is_inside(link, -100, 0, 13.5)


@pytest.mark.slow
def test_wrist_motor_clears_the_wall_and_the_slot(link):
    motor, board = in_host("nema17_40mm#3"), in_host("mks_servo42d#3")
    assert motor.bounding_box().max.X + R.plug_clearance <= R.wall_x[0] + 1e-6
    assert interference(motor, link) < 1.0 and interference(board, link) < 1.0
    cap1 = parts.build("j2_cap_1")
    assert interference(motor, cap1) < 1.0 and interference(board, cap1) < 1.0
    # the window clears the body by >= 2 mm all round
    x0, x1, half_w = motor_window(DEFAULT)
    bb = motor.bounding_box()
    assert bb.min.X - x0 >= 2.0 - 1e-6 and x1 - bb.max.X >= 2.0 - 1e-6 and half_w - max(abs(bb.min.Y), abs(bb.max.Y)) >= 2.0 - 1e-6


@pytest.mark.slow
def test_caps_still_build_and_end_at_the_wall():
    """The caps are slated for removal (docs/open_issues.md) - until then they must not run into the wall."""
    for name in ("j2_cap_1", "j2_cap_2"):
        cap = parts.build(name)
        assert cap.is_valid and len(cap.solids()) == 1
        assert cap.bounding_box().max.X <= R.wall_x[0] + 1e-6, name
