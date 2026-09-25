"""The DEFAULT forearm - the roll end: the flange wall replaces the elbow disc, the wrist-pitch motor sits where
the stock belt puts it and clears the wall, the wrist end is the SolidWorks one, the removed caps' sockets are gone."""
import math
from dataclasses import replace

import pytest
from build123d import Box, Pos

import parts
from lib import params as PARAMS
from lib.belts import GT2_PULLEY_20T_TEETH, GT2_PULLEY_90T_TEETH, closed_belt_length
from lib.forearm import DEFAULT, LEGACY, flange_bolt_points, link_socket_points
from lib.motors import NEMA17_40_BODY_W, NEMA17_40_CONNECTOR_D
from tests.forearm.helpers import in_host, interference, is_inside

R = DEFAULT.roll_end


def test_default_layout():
    assert DEFAULT.roll and not LEGACY.roll
    assert DEFAULT.web == LEGACY.web and DEFAULT.boss == LEGACY.boss
    assert DEFAULT.sockets is None and LEGACY.sockets is not None and link_socket_points(DEFAULT) == ([], [])
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
    assert len(flange_bolt_points(DEFAULT)) == 4 and (R.bolt_circle_dia / 2.0, 25.0) in flange_bolt_points(DEFAULT)
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
    # the wrist end (x < -165: the boss) is the SolidWorks one, but for the sockets
    probe = Pos(-210.0, 0.0, 15.0) * Box(90.0, 100.0, 60.0)
    assert abs(interference(link, probe) - interference(build_link(replace(LEGACY, sockets=None)), probe)) < 0.5
    # no locating sockets: the caps they held are gone
    assert is_inside(link, -120, 40, 18) and is_inside(link, -120, 40, 9) and is_inside(link, -244.64, 20, 9)
    # the wall: solid, the flange recess on its elbow face, the cable bore and the bolt holes through it
    xm, x_face = (R.wall_x[0] + R.wall_x[1]) / 2.0, R.wall_x[1]
    assert is_inside(link, xm, 40, 50) and is_inside(link, xm, 0, -8)
    rr, rb = (R.flange_dia + R.flange_recess_add) / 2.0, R.cable_bore / 2.0
    assert not is_inside(link, x_face - 1.0, 0, 25 + rr - 1.0) and is_inside(link, x_face - 3.0, 0, 25 + rr - 1.0)   # the spigot recess, 2 deep
    assert not is_inside(link, xm, 0, 25) and not is_inside(link, xm, 0, 25 + rb - 1.0)               # the cable bore
    assert is_inside(link, xm, 0, 25 + rb + 1.5)
    for y, z in flange_bolt_points(DEFAULT):
        assert not is_inside(link, xm, y, z) and not is_inside(link, R.wall_x[0] + 0.1, y, z)
    # the slots: the central one passes the pilot, both stop before the wall
    assert not is_inside(link, DEFAULT.motor_x, 11.0, 13.5) and is_inside(link, DEFAULT.motor_x, 11.3, 13.5)
    assert is_inside(link, R.wall_x[0] - 4.0, 0, 13.5)


@pytest.mark.slow
def test_wrist_motor_clears_the_wall_and_the_slot(link):
    motor, board = in_host("nema17_40mm#3"), in_host("mks_servo42d#3")
    assert motor.bounding_box().max.X + R.plug_clearance <= R.wall_x[0] + 1e-6
    assert interference(motor, link) < 1.0 and interference(board, link) < 1.0
