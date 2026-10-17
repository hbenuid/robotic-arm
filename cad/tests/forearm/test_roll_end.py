"""The DEFAULT forearm - the roll end: the flange wall replaces the elbow disc (a round flange on the roll axis on a
foot as wide as the straight web, cut flush with the web's belt face but for a boss round the bottom screw, braced by
two gussets beside the wrist motor), the wrist-pitch motor sits where the stock belt puts it, clears the wall's screw
heads and has its plug's room beside it, the wrist end SHORTENING nearer the elbow - the SolidWorks bearing seat in a
boss as wide as the web, ending at the seat's top under j3_coupler -, the removed caps' sockets are gone."""
import math
from dataclasses import replace

import pytest
from build123d import Box, Location

from lib import mounts
from lib import params as PARAMS
from lib.belts import GT2_PULLEY_20T_TEETH, GT2_PULLEY_90T_TEETH, closed_belt_length
from lib.datum import to_location
from lib.forearm import (
    DEFAULT,
    LEGACY,
    elbow_end_x,
    flange_bolt_points,
    link_socket_points,
    screw_channel,
    screws_under_the_web,
)
from lib.forearm.params import SHORTENING
from lib.motors import (
    MKS_SERVO42D_W,
    NEMA17_40_BODY_W,
    NEMA17_40_CONNECTOR_D,
    NEMA17_40_CONNECTOR_W,
    NEMA17_40_CONNECTOR_Z0,
    NEMA17_40_CONNECTOR_Z1,
)
from tests import built
from tests.forearm.helpers import in_host
from tests.helpers import interference, is_inside

R = DEFAULT.roll_end


def test_default_layout():
    assert DEFAULT.roll and not LEGACY.roll
    assert DEFAULT.web == replace(LEGACY.web, wrist_x=LEGACY.web.wrist_x + SHORTENING, half_w=R.wall_od / 2.0) and SHORTENING > 0.0
    # the wrist boss: the SolidWorks seat and lip, as wide as the web, its top the seat's top, no recess
    b, lb = DEFAULT.boss, LEGACY.boss
    assert (b.seat_dia, b.seat_z1, b.lip_dia, b.lip_z) == (lb.seat_dia, lb.seat_z1, lb.lip_dia, lb.lip_z)
    assert b.dia == 2.0 * DEFAULT.web.half_w and b.z1 == b.seat_z1 and b.recess_dia is None and lb.recess_dia is not None
    assert DEFAULT.sockets is None and LEGACY.sockets is not None and link_socket_points(DEFAULT) == ([], [])
    # the slide: set by the belt (no idler: tests/forearm/test_wrist_idler.py), inside the (shortened) slots, the body
    # clear of the wall screws' heads at the slots' wall end (the connector turned sideways:
    # test_wrist_motor_plug_room_is_clear) and of the boss
    x = DEFAULT.motor_x
    assert math.isclose(closed_belt_length(x - DEFAULT.web.wrist_x, GT2_PULLEY_90T_TEETH, GT2_PULLEY_20T_TEETH), R.wrist_belt, abs_tol=1e-6)
    lo, hi = DEFAULT.slide_range
    assert lo < x < hi
    assert hi + NEMA17_40_BODY_W / 2.0 + R.screw.head_h + R.motor_wall_clear <= R.wall_x[0] + 1e-9
    assert lo - NEMA17_40_BODY_W / 2.0 >= DEFAULT.web.wrist_x + DEFAULT.boss.dia / 2.0
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


def test_the_straight_web_the_wall_and_its_gussets():
    """The web runs straight from the wall to its round wrist end, as wide as the wall's round flange - over the recess
    with room to spare, and over the roll pulley's ring it bolts onto - and the wall's foot; the wall stops at the web's
    belt face but for the boss round the bottom screw, whose hole would break out there (the boss no lower than the
    recess); the gussets stand beside the wrist motor and its board, inside the web."""
    D, w, s = DEFAULT.drive, DEFAULT.web, DEFAULT.slot
    assert w.half_w == R.wall_od / 2.0 == DEFAULT.boss.dia / 2.0 and PARAMS.FOREARM_WALL_OD == R.wall_od
    assert R.wall_od / 2.0 >= (R.flange_dia + R.flange_recess_add) / 2.0 + 8.0
    assert R.wall_od >= D.ring_flange_dia
    # the motor's footprint on the web over the whole slide: its body and the side slots 3 mm inside the web's sides
    assert w.half_w >= NEMA17_40_BODY_W / 2.0 + 3.0 and w.half_w >= s.side_y + s.side_w / 2.0 + 3.0
    # the bottom screw's boss
    (_, z_low), = screws_under_the_web(DEFAULT)
    hole_bottom, boss_bottom = z_low - R.bolt_dia / 2.0, z_low - R.screw_boss_dia / 2.0
    assert hole_bottom < w.z0 and hole_bottom - boss_bottom >= 2.5
    assert boss_bottom >= R.axis_z - (R.flange_dia + R.flange_recess_add) / 2.0 - 1e-9
    # the gussets: outside the motor's body and board, inside the web, rooted on the wall; the round outline tops them
    assert R.rib_y[0] >= max(NEMA17_40_BODY_W, MKS_SERVO42D_W) / 2.0 + 2.0 and R.rib_y[1] <= w.half_w
    assert w.z1 + R.rib_h <= R.axis_z + math.sqrt((R.wall_od / 2.0) ** 2 - R.rib_y[0] ** 2)


@pytest.fixture(scope="module")
def link():
    return built.part("j2_link")


@pytest.mark.slow
def test_link_ends_at_the_wall_and_at_its_wrist_boss(link):
    w, b = DEFAULT.web, DEFAULT.boss
    (_, z_low), = screws_under_the_web(DEFAULT)
    boss_bottom = z_low - R.screw_boss_dia / 2.0
    assert link.is_valid and len(link.solids()) == 1
    bb = link.bounding_box()
    assert math.isclose(bb.max.X, R.wall_x[1], abs_tol=1e-6) and math.isclose(bb.min.X, w.wrist_x - w.half_w, abs_tol=1e-6)
    assert math.isclose(bb.min.Z, boss_bottom, abs_tol=1e-6) and math.isclose(bb.max.Z, R.axis_z + R.wall_od / 2.0, abs_tol=1e-6)
    assert math.isclose(bb.max.Y, w.half_w, abs_tol=1e-6) and math.isclose(bb.min.Y, -w.half_w, abs_tol=1e-6)
    # the wrist end: round, as wide as the web; the boss's top the seat's top; the seat below and above the lip
    wx = w.wrist_x
    for dx, dy in ((-1.0, 0.0), (-math.sqrt(0.5), math.sqrt(0.5)), (0.0, -1.0)):
        assert is_inside(link, wx + dx * (w.half_w - 0.3), dy * (w.half_w - 0.3), 13.5)
        assert not is_inside(link, wx + dx * (w.half_w + 0.3), dy * (w.half_w + 0.3), 13.5)
    assert is_inside(link, wx + 25.0, 0, b.z1 - 0.3) and not is_inside(link, wx + 25.0, 0, b.z1 + 0.3)
    for z in (w.z0 + 1.0, b.seat_z1 - 1.0):
        assert not is_inside(link, wx + b.seat_dia / 2.0 - 0.3, 0, z) and is_inside(link, wx + b.seat_dia / 2.0 + 0.3, 0, z)
    assert is_inside(link, wx + b.lip_dia / 2.0 + 0.3, 0, sum(b.lip_z) / 2.0) and not is_inside(link, wx + b.lip_dia / 2.0 - 0.3, 0, sum(b.lip_z) / 2.0)
    # ... under j3_coupler#2: its dust lip turns 0.5 over the boss's top
    assert link.distance_to(in_host("j3_coupler#2")) >= 0.5 - 1e-6
    # no locating sockets (the caps they held are gone): the web whole on both faces out to its straight sides
    assert is_inside(link, -120, w.half_w - 1.0, w.z1 - 1.0) and is_inside(link, -120, w.half_w - 1.0, w.z0 + 1.0)
    assert not is_inside(link, -120, w.half_w + 1.0, 13.5)                                               # the web's side
    # the wall: a round flange about the roll axis on its foot, cut at the web's belt face but for the bottom screw's
    # boss; the flange recess on its elbow face, the cable bore and the bolt holes through it
    xm, x_face, top, foot = (R.wall_x[0] + R.wall_x[1]) / 2.0, R.wall_x[1], R.axis_z + R.wall_od / 2.0, w.half_w
    assert is_inside(link, xm, 0, top - 1.0) and not is_inside(link, xm, 0, top + 1.0) and not is_inside(link, xm, 0, 2 * R.axis_z - top + 1.0)
    assert is_inside(link, xm, foot - 1.0, w.z0 + 1.0) and not is_inside(link, xm, foot + 1.0, w.z0 + 1.0)
    assert not is_inside(link, xm, R.screw_boss_dia / 2.0 + 1.0, w.z0 - 0.5) and not is_inside(link, xm, foot - 1.0, w.z0 - 0.5)   # the L
    assert is_inside(link, xm, 0, boss_bottom + 0.5) and not is_inside(link, xm, 0, boss_bottom - 0.5)                            # the boss
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
    _, x1, half_w, z_top = screw_channel(z_low)
    xc = (DEFAULT.slot.centre_x[1] + DEFAULT.slot.centre_w / 2.0 + x1) / 2.0     # between the central slot's end and the wall
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
def test_wrist_motor_plug_room_is_clear():
    """The placed motor's connector faces sideways (j2_link's -Y) and the plug_clearance of room beyond its boss - the
    cable's plug - stays clear of j2_link (its web, gussets and wall) and of the motor's own board (the motor and its
    board clear of j2_link, the pilot in the central slot: tests/test_mounts.py
    test_motors_and_boards_clear_their_neighbours)."""
    motor = in_host("nema17_40mm#3")
    bb = motor.bounding_box()
    out = NEMA17_40_BODY_W / 2.0 + NEMA17_40_CONNECTOR_D
    assert math.isclose(bb.min.Y, -out, abs_tol=0.05) and math.isclose(bb.max.Y, NEMA17_40_BODY_W / 2.0, abs_tol=0.05)
    frame = to_location(mounts.BY_KEY["nema17_40mm#3"].frame)
    room = Box(NEMA17_40_CONNECTOR_W, R.plug_clearance, NEMA17_40_CONNECTOR_Z1 - NEMA17_40_CONNECTOR_Z0).moved(
        frame * Location((0.0, -out - R.plug_clearance / 2.0, (NEMA17_40_CONNECTOR_Z0 + NEMA17_40_CONNECTOR_Z1) / 2.0)))
    for key, other in (("j2_link", built.part("j2_link")), ("mks_servo42d#3", in_host("mks_servo42d#3"))):
        assert interference(room, other) == 0.0 and room.distance_to(other) > 1.0, key
