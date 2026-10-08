"""The DEFAULT forearm - the roll end: the flange wall replaces the elbow disc (a round flange on the roll axis, on
the foot the web necks down to, braced by two gussets beside the wrist motor), the wrist-pitch motor sits where the
stock belt puts it and clears the wall, the wrist end is the SolidWorks one SHORTENING nearer the elbow, the removed
caps' sockets are gone."""
import math
from dataclasses import replace

import pytest
from build123d import Box, Pos

from lib import params as PARAMS
from lib.belts import GT2_PULLEY_20T_TEETH, GT2_PULLEY_90T_TEETH, closed_belt_length
from lib.forearm import (
    DEFAULT,
    LEGACY,
    elbow_end_x,
    flange_bolt_points,
    link_socket_points,
    neck_tangent,
    screw_channel,
    screws_under_the_web,
    web_half_width,
)
from lib.forearm.params import SHORTENING
from lib.motors import MKS_SERVO42D_W, NEMA17_40_BODY_W, NEMA17_40_CONNECTOR_D
from tests import built
from tests.forearm.helpers import in_host
from tests.helpers import interference, is_inside

R = DEFAULT.roll_end


def test_default_layout():
    assert DEFAULT.roll and not LEGACY.roll
    assert DEFAULT.web == replace(LEGACY.web, wrist_x=LEGACY.web.wrist_x + SHORTENING) and SHORTENING > 0.0
    assert DEFAULT.boss == LEGACY.boss
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


def test_the_wall_screws_way_in():
    """Only the bottom screw's head would land in the web: its channel opens to the web's underside (the screw lays in
    from the belt side), runs from the wall into the central motor slot (a key reaches the head) and leaves the web whole
    over it; every other head clears the web's top face."""
    w, s = DEFAULT.web, DEFAULT.slot
    reach = R.screw.head_dia / 2.0 + R.channel_clear
    under = screws_under_the_web(DEFAULT)
    assert under == [(y, z) for y, z in flange_bolt_points(DEFAULT) if z < R.axis_z and abs(y) < 1e-9]
    assert all(z - reach >= w.z1 for y, z in flange_bolt_points(DEFAULT) if (y, z) not in under)
    (_, z_low), = under
    x0, x1, half_w, z_top = screw_channel(z_low)
    assert z_low - reach < w.z0 and x1 == elbow_end_x(DEFAULT) and x0 == s.centre_x[1] and half_w == reach
    assert x1 - x0 >= R.screw_len + R.screw.head_h and w.z1 - z_top >= 5.0


def test_the_neck_the_round_wall_and_its_gussets():
    """The wall is a round flange about the roll axis - over the recess with room to spare, and over the roll pulley's
    ring it bolts onto - standing on a foot as wide as itself, which the web necks down to from the wrist boss (its sides
    tangent to the boss); the gussets stand beside the wrist motor and its board, inside the neck."""
    D, w, b = DEFAULT.drive, DEFAULT.web, DEFAULT.boss
    assert R.neck_half_w == R.wall_od / 2.0 < w.half_w and PARAMS.FOREARM_WALL_OD == R.wall_od
    assert R.wall_od / 2.0 >= (R.flange_dia + R.flange_recess_add) / 2.0 + 8.0
    assert R.wall_od >= D.ring_flange_dia
    # the neck: tangent to the boss, widening all the way from the wall; the motor's footprint on it over the whole slide
    tx, ty = neck_tangent(DEFAULT)
    px, py = elbow_end_x(DEFAULT), R.neck_half_w
    assert math.isclose(math.hypot(tx - w.wrist_x, ty), b.dia / 2.0, abs_tol=1e-9)
    assert abs((px - tx) * (tx - w.wrist_x) + (py - ty) * ty) < 1e-6                     # the side is the boss' tangent there
    assert w.wrist_x < tx < DEFAULT.slide_range[0] - NEMA17_40_BODY_W / 2.0 and py < ty <= w.half_w
    assert math.isclose(web_half_width(px), py) and math.isclose(web_half_width(tx), ty)
    for x in DEFAULT.slot.side_x:
        assert web_half_width(x) >= NEMA17_40_BODY_W / 2.0 + 3.0
    # the gussets: outside the motor's body and board, inside the neck, rooted on the wall; the round outline tops them
    assert R.rib_y[0] >= max(NEMA17_40_BODY_W, MKS_SERVO42D_W) / 2.0 + 2.0 and R.rib_y[1] <= R.neck_half_w
    assert w.z1 + R.rib_h <= R.axis_z + math.sqrt((R.wall_od / 2.0) ** 2 - R.rib_y[0] ** 2)


@pytest.fixture(scope="module")
def link():
    return built.part("j2_link")


@pytest.mark.slow
def test_link_ends_at_the_wall_and_keeps_its_wrist_end(link):
    from lib.forearm.link import build_link

    assert link.is_valid and len(link.solids()) == 1
    bb = link.bounding_box()
    assert math.isclose(bb.max.X, R.wall_x[1], abs_tol=1e-6) and math.isclose(bb.min.Z, R.axis_z - R.wall_od / 2.0, abs_tol=1e-6)
    assert math.isclose(bb.max.Z, R.axis_z + R.wall_od / 2.0, abs_tol=1e-6)
    # the wrist end (the boss's outer half, beyond the pivot) is the SolidWorks one SHORTENING nearer the elbow, but
    # for the sockets
    probe = Box(45.0, 100.0, 60.0)
    at_default, at_legacy = Pos(DEFAULT.web.wrist_x - 22.5, 0.0, 15.0), Pos(LEGACY.web.wrist_x - 22.5, 0.0, 15.0)
    legacy = build_link(replace(LEGACY, sockets=None))
    assert abs(interference(link, at_default * probe) - interference(legacy, at_legacy * probe)) < 0.5
    # no locating sockets: the caps they held are gone
    wrist_socket_x = DEFAULT.web.wrist_x + LEGACY.sockets.wrist[0][0] - LEGACY.web.wrist_x
    y_side = web_half_width(-120.0)
    assert is_inside(link, -120, y_side - 1.0, 18) and is_inside(link, -120, y_side - 1.0, 9) and is_inside(link, wrist_socket_x, 20, 9)
    assert not is_inside(link, -120, y_side + 1.0, 13.5)                                                 # the neck's side
    # the wall: a round flange about the roll axis on its foot, the belly under the web; the flange recess on its
    # elbow face, the cable bore and the bolt holes through it
    xm, x_face, top, foot = (R.wall_x[0] + R.wall_x[1]) / 2.0, R.wall_x[1], R.axis_z + R.wall_od / 2.0, R.neck_half_w
    w = DEFAULT.web
    assert is_inside(link, xm, 0, top - 1.0) and not is_inside(link, xm, 0, top + 1.0) and is_inside(link, xm, 0, 2 * R.axis_z - top + 1.0)
    assert is_inside(link, xm, foot - 1.0, w.z0 + 1.0) and not is_inside(link, xm, foot + 1.0, w.z0 + 1.0)
    assert not is_inside(link, xm, 40, 50) and not is_inside(link, xm, foot - 1.0, top - 1.0)             # no corners
    rr, rb = (R.flange_dia + R.flange_recess_add) / 2.0, R.cable_bore / 2.0
    assert not is_inside(link, x_face - 1.0, 0, 25 + rr - 1.0) and is_inside(link, x_face - 3.0, 0, 25 + rr - 1.0)   # the spigot recess, 2 deep
    assert not is_inside(link, xm, 0, 25) and not is_inside(link, xm, 0, 25 + rb - 1.0)               # the cable bore
    assert is_inside(link, xm, 0, 25 + rb + 1.5)
    # every screw goes in from the wrist side: its head's room behind the wall is open (the bottom one's in the channel
    # under the web, open to the underside, the web whole over it)
    head_r = R.screw.head_dia / 2.0
    for y, z in flange_bolt_points(DEFAULT):
        assert not is_inside(link, xm, y, z) and not is_inside(link, R.wall_x[0] + 0.1, y, z)
        for dy, dz in ((0.0, 0.0), (head_r, 0.0), (-head_r, 0.0), (0.0, head_r), (0.0, -head_r)):
            assert not is_inside(link, R.wall_x[0] - 1.0, y + dy, z + dz)
            assert not is_inside(link, R.wall_x[0] - R.screw.head_h - 1.0, y + dy, z + dz)
    (y_low, z_low), = screws_under_the_web(DEFAULT)
    x0, x1, half_w, z_top = screw_channel(z_low)
    xc = (x0 + x1) / 2.0
    assert not is_inside(link, xc, y_low, w.z0 + 0.5) and not is_inside(link, xc, y_low + half_w - 0.3, z_top - 0.3)
    assert is_inside(link, xc, y_low, z_top + 0.3) and is_inside(link, xc, y_low + half_w + 0.3, w.z0 + 0.5)
    # the slots: the central one passes the pilot, both stop before the wall
    assert not is_inside(link, DEFAULT.motor_x, 11.0, 13.5) and is_inside(link, DEFAULT.motor_x, 11.3, 13.5)
    assert is_inside(link, R.wall_x[0] - 4.0, 0, 13.5)
    # the gussets: either side against the wall's wrist face, tapering to the web along rib_len; nothing between them
    y_rib, z_web = sum(R.rib_y) / 2.0, w.z1
    for side in (1.0, -1.0):
        assert is_inside(link, R.wall_x[0] - 1.0, side * y_rib, z_web + R.rib_h - 2.0)
        assert not is_inside(link, R.wall_x[0] - R.rib_len + 2.0, side * y_rib, z_web + 2.0)
        assert is_inside(link, R.wall_x[0] - R.rib_len + 4.0, side * y_rib, z_web + 0.5)
        assert not is_inside(link, R.wall_x[0] - 1.0, side * (R.rib_y[0] - 1.0), z_web + 2.0)
    assert not is_inside(link, R.wall_x[0] - 1.0, 0, z_web + 2.0)


@pytest.mark.slow
def test_wrist_motor_plug_clears_the_wall():
    """The placed motor's connector end plug_clearance short of the wall (the motor and its board clear of j2_link,
    the pilot in the central slot: tests/test_mounts.py test_motors_and_boards_clear_their_neighbours)."""
    motor = in_host("nema17_40mm#3")
    assert motor.bounding_box().max.X + R.plug_clearance <= R.wall_x[0] + 1e-6
