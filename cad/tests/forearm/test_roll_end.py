"""The DEFAULT forearm - the roll end: the flange wall replaces the elbow disc (a faceted flange on the roll axis through
its whole thickness, whole all round under the roll pulley's ring, its corners inside the roll tensioner's screw head's
path; the wider web grows out of it through the flange's lower diagonal facets carried on along it into the web's edge
chamfer; braced by two gussets beside the wrist motor), the web and the boss WEB_DROP toward the belt
side - the web's underside level with the wall's bottom flat, every wall screw's head over the web -, the web as wide as
its belt cover's end round the wrist's 90T (docs/design_language.md), the wrist-pitch
motor sits on round holes WRIST_MOTOR_CENTRES from the wrist, clears the wall's screw heads and has its plug's room
beside it, the wrist end SHORTENING nearer the elbow - the SolidWorks bearing seat in a faceted boss as wide as the web,
ending at the seat's top under wrist_link's flange -, the removed caps' sockets are gone."""
import math
from dataclasses import replace

import pytest
from build123d import Box, Location

from lib import mounts
from lib import params as PARAMS
from lib.belts import GT2_IDLER_WIDTH, GT2_PULLEY_20T_TEETH, GT2_PULLEY_90T_TEETH, closed_belt_length
from lib.datum import to_location
from lib.forearm import DEFAULT, LEGACY, flange_bolt_points, link_socket_points, screws_under_the_web
from lib.forearm.params import BOSS_TRIM, PULLEY_90T, SHORTENING, WEB_DROP, WRIST_MOTOR_CENTRES, RollEndParams
from lib.motors import (
    MKS_SERVO42D_W,
    NEMA17_40_BODY_LEN,
    NEMA17_40_BODY_W,
    NEMA17_40_CONNECTOR_D,
    NEMA17_40_CONNECTOR_W,
    NEMA17_40_CONNECTOR_Z0,
    NEMA17_40_CONNECTOR_Z1,
)
from lib.style import COVER_CLEAR, COVER_WALL, EDGE_CHAMFER, SIDES
from tests import built
from tests.forearm.helpers import in_host
from tests.helpers import interference, is_inside

R = DEFAULT.roll_end


def test_default_layout():
    assert DEFAULT.roll and not LEGACY.roll
    lw = LEGACY.web
    assert DEFAULT.web == replace(lw, wrist_x=lw.wrist_x + SHORTENING, half_w=DEFAULT.web.half_w, z0=lw.z0 - WEB_DROP,
                                  z1=lw.z1 - WEB_DROP) and SHORTENING > 0.0
    # as wide as its belt cover's end: the wrist's 90T turning inside it, COVER_CLEAR off it, inside COVER_WALL
    assert DEFAULT.web.half_w == pytest.approx(PULLEY_90T.flange_dia / 2.0 + COVER_CLEAR + COVER_WALL, abs=1e-6)
    assert DEFAULT.web.half_w > R.wall_od / 2.0
    # the web dropped toward its belt side until its underside is level with the wall's round bottom, the wrist belt
    # with it
    assert DEFAULT.web.z0 == R.axis_z - R.wall_od / 2.0 and WEB_DROP > 0.0 and DEFAULT.web.thickness == lw.thickness
    assert R.wrist_belt_z == RollEndParams().wrist_belt_z - WEB_DROP == PARAMS.J2_BELT_Z
    # the wrist boss: the SolidWorks seat and lip dropped with the web, as wide as it, its top the seat's top - trimmed
    # BOSS_TRIM, flush with the upper wrist bearing (its 7 on the lip) -, no recess
    b, lb = DEFAULT.boss, LEGACY.boss
    assert (b.seat_dia, b.lip_dia) == (lb.seat_dia, lb.lip_dia)
    assert (b.seat_z1, b.lip_z) == (lb.seat_z1 - WEB_DROP - BOSS_TRIM, (lb.lip_z[0] - WEB_DROP, lb.lip_z[1] - WEB_DROP))
    assert b.seat_z1 - b.lip_z[1] == PARAMS.BEARING_6806_WIDTH and BOSS_TRIM == 1.5
    assert b.dia == 2.0 * DEFAULT.web.half_w and b.z1 == b.seat_z1 and b.recess_dia is None and lb.recess_dia is not None
    assert DEFAULT.sockets is None and LEGACY.sockets is not None and link_socket_points(DEFAULT) == ([], [])
    # the motor on round holes WRIST_MOTOR_CENTRES from the wrist, nearer than the stock belt's own centre distance (its
    # idler takes up the rest: tests/forearm/test_wrist_idler.py), the body clear of the wall screws' heads (the connector
    # turned sideways: test_wrist_motor_plug_room_is_clear) and of the boss; the pilot hole >= 4 mm before the wall
    x, h = DEFAULT.motor_x, DEFAULT.holes
    assert x == DEFAULT.web.wrist_x + WRIST_MOTOR_CENTRES and DEFAULT.slot is None and DEFAULT.slide_range is None
    assert closed_belt_length(WRIST_MOTOR_CENTRES, GT2_PULLEY_90T_TEETH, GT2_PULLEY_20T_TEETH) < R.wrist_belt
    assert x + NEMA17_40_BODY_W / 2.0 + R.screw.head_h + R.motor_wall_clear <= R.wall_x[0] + 1e-9
    assert x - NEMA17_40_BODY_W / 2.0 >= DEFAULT.web.wrist_x + DEFAULT.boss.dia / 2.0
    assert h.pilot_dia >= 22.3 and h.bolt_half == 15.5 and x + h.pilot_dia / 2.0 <= R.wall_x[0] - 4.0
    assert len(flange_bolt_points(DEFAULT)) == 4 and (R.bolt_circle_dia / 2.0, 25.0) in flange_bolt_points(DEFAULT)
    assert PARAMS.J2_MOTOR_X == x and PARAMS.FOREARM_ROLL_AXIS_Z == R.axis_z


def test_the_wall_screws_way_in():
    """No wall screw's head lands in the web: the web dropped under the bottom one, so every head - channel_clear round
    it - clears the web's top face and none needs a channel under the web (screws_under_the_web); the bottom one's
    head room just over the web."""
    w = DEFAULT.web
    reach = R.screw.head_dia / 2.0 + R.channel_clear
    assert screws_under_the_web(DEFAULT) == []
    assert all(z - reach >= w.z1 for _, z in flange_bolt_points(DEFAULT))
    assert min(z for _, z in flange_bolt_points(DEFAULT)) - reach - w.z1 == pytest.approx(0.25)


def test_the_web_grows_out_of_the_flange():
    """The web is wider than the flange, so at the wall its corners lie outside the flange's lower diagonal facets: no
    foot fills them (its corners would meet the roll tensioner's screw head as the forearm rolls - the flange's own
    corners stay inside its path: tests/forearm/test_roll_drive.py), the facets carry on along the web instead, over the
    gussets' length, down to the web's edge chamfer - the octagon's facets at 45 deg like the chamfer, so they run out
    into it; the gussets' inner faces stand on the facets' top edge at the wall, their outer faces under the side flats."""
    w = DEFAULT.web
    apo = R.wall_od / 2.0
    corner_y = apo * math.tan(math.pi / SIDES)                      # the bottom flat's corner
    assert SIDES == 8 and w.z0 == R.axis_z - apo                    # the facet at 45 deg; the web on the bottom flat
    assert apo / math.cos(math.pi / SIDES) > DEFAULT.drive.ring_flange_dia / 2.0   # the flange whole under the ring
    assert w.half_w - EDGE_CHAMFER > corner_y + w.thickness         # the web's corners outside the diagonal at the wall
    assert R.facet_len == R.rib_len
    assert R.rib_y == pytest.approx((corner_y + w.thickness, apo), abs=1e-6)


def test_the_straight_web_the_wall_and_its_gussets():
    """The web runs straight from the wall to its faceted wrist end, wider than the wall's faceted flange (the flange
    over the recess with room to spare, and over the roll pulley's ring it bolts onto, all round) - the wall's foot as
    wide as the web -; the gussets stand beside the wrist motor and its board, inside the web."""
    D, w, h = DEFAULT.drive, DEFAULT.web, DEFAULT.holes
    assert w.half_w == DEFAULT.boss.dia / 2.0 > R.wall_od / 2.0 and PARAMS.FOREARM_WALL_OD == R.wall_od
    assert R.wall_od / 2.0 >= (R.flange_dia + R.flange_recess_add) / 2.0 + 8.0
    assert R.wall_od >= D.ring_flange_dia
    # the motor's footprint on the web: its body and its bolt holes 3 mm inside the web's sides
    assert w.half_w >= NEMA17_40_BODY_W / 2.0 + 3.0 and w.half_w >= h.bolt_half + h.bolt_dia / 2.0 + 3.0
    # the gussets: outside the motor's body, under its plug and its board (on the body's back: wider than the body, above
    # the gussets' tops), inside the web, rooted on the wall under the flange's side flats
    assert R.rib_y[0] >= NEMA17_40_BODY_W / 2.0 + 2.0 and R.rib_y[1] <= w.half_w
    assert R.rib_h < -NEMA17_40_CONNECTOR_Z1 < NEMA17_40_BODY_LEN and MKS_SERVO42D_W > NEMA17_40_BODY_W
    assert w.z1 + R.rib_h <= R.axis_z + R.wall_od / 2.0 * math.tan(math.pi / SIDES)


@pytest.fixture(scope="module")
def link():
    return built.part("j2_link")


@pytest.mark.slow
def test_link_ends_at_the_wall_and_at_its_wrist_boss(link):
    w, b = DEFAULT.web, DEFAULT.boss
    assert link.is_valid and len(link.solids()) == 1
    bb = link.bounding_box()
    assert math.isclose(bb.max.X, R.wall_x[1], abs_tol=1e-6) and math.isclose(bb.min.X, w.wrist_x - w.half_w, abs_tol=1e-6)
    # lowest: the wrist idler's rib (from the idler's top face up to the web), under the wall's bottom flat
    z_rib = R.wrist_belt_z + GT2_IDLER_WIDTH / 2.0
    assert math.isclose(bb.min.Z, z_rib, abs_tol=1e-6) and z_rib < R.axis_z - R.wall_od / 2.0
    assert math.isclose(bb.max.Z, R.axis_z + R.wall_od / 2.0, abs_tol=1e-6)
    assert math.isclose(bb.max.Y, w.half_w, abs_tol=1e-6) and math.isclose(bb.min.Y, -w.half_w, abs_tol=1e-6)
    # the wrist end: faceted, as wide as the web (its end, diagonal and side flats); the boss's top the seat's top; the seat below and above the lip
    wx, zm = w.wrist_x, (w.z0 + w.z1) / 2.0
    for dx, dy in ((-1.0, 0.0), (-math.sqrt(0.5), math.sqrt(0.5)), (0.0, -1.0)):
        assert is_inside(link, wx + dx * (w.half_w - 0.3), dy * (w.half_w - 0.3), zm)
        assert not is_inside(link, wx + dx * (w.half_w + 0.3), dy * (w.half_w + 0.3), zm)
    assert is_inside(link, wx + 25.0, 0, b.z1 - 0.3) and not is_inside(link, wx + 25.0, 0, b.z1 + 0.3)
    for z in (w.z0 + 1.0, b.seat_z1 - 1.0):
        assert not is_inside(link, wx + b.seat_dia / 2.0 - 0.3, 0, z) and is_inside(link, wx + b.seat_dia / 2.0 + 0.3, 0, z)
    assert is_inside(link, wx + b.lip_dia / 2.0 + 0.3, 0, sum(b.lip_z) / 2.0) and not is_inside(link, wx + b.lip_dia / 2.0 - 0.3, 0, sum(b.lip_z) / 2.0)
    # ... under wrist_link's flange (j3_coupler#2 retired): its coupler face FACE_GAP (2) over the boss's face, the post's
    # chamfered foot over the seat's mouth, 1.9 or more off the rim
    assert link.distance_to(in_host("wrist_link#1")) >= 1.9
    # no locating sockets (the caps they held are gone): the web whole on both faces out to its straight sides
    assert is_inside(link, -120, w.half_w - 1.0, w.z1 - 1.0) and is_inside(link, -120, w.half_w - 1.0, w.z0 + 1.0)
    assert not is_inside(link, -120, w.half_w + 1.0, zm)                                                 # the web's side
    # the wall: the faceted flange about the roll axis through its whole thickness, its bottom flat level with the web's
    # underside (the roll pulley's ring bears on it all round, the recess closed); the flange recess on its elbow face,
    # the cable bore and the bolt holes through it; no foot - the flange's corners, the lower ones too, at every station
    xm, x_face, top, foot = (R.wall_x[0] + R.wall_x[1]) / 2.0, R.wall_x[1], R.axis_z + R.wall_od / 2.0, w.half_w
    bottom, apo = 2 * R.axis_z - top, R.wall_od / 2.0
    assert bottom == w.z0
    assert is_inside(link, xm, 0, top - 1.0) and not is_inside(link, xm, 0, top + 1.0)
    assert is_inside(link, xm, 0, bottom + 1.0) and not is_inside(link, xm, 0, bottom - 1.0)                # the bottom
    assert is_inside(link, x_face - 0.5, 0, R.axis_z - (R.flange_dia + R.flange_recess_add) / 2.0 - 1.0)   # ... under the recess
    assert not is_inside(link, xm, 40, 50) and not is_inside(link, xm, foot - 1.0, top - 1.0)             # no corners
    r_corner = apo / math.cos(math.pi / SIDES)
    for x in (R.wall_x[0] + 0.3, xm, x_face - 0.3):
        for k in range(SIDES):   # the flange's corners, from the bottom flat's on round
            a = -math.pi / 2.0 + math.pi / SIDES + 2.0 * math.pi * k / SIDES
            for r, inside in ((r_corner - 0.5, True), (r_corner + 0.3, False)):
                assert is_inside(link, x, r * math.cos(a), R.axis_z + r * math.sin(a)) is inside, (x, k, r)
        assert not is_inside(link, x, foot - 1.0, w.z0 + 1.0)                                              # no foot
    # the web grows out of it through the lower facets carried on along it: at the wall the flange's own diagonal, half
    # way along the plane between it and the web's edge chamfer, past facet_len the web's whole section
    corner_y = apo * math.tan(math.pi / SIDES)
    drop = foot - EDGE_CHAMFER - corner_y
    for side in (1.0, -1.0):
        for t in (0.02, 0.5):
            x, z = R.wall_x[0] - t * R.facet_len, w.z0 + 2.0
            y = corner_y + (z - w.z0) + t * drop        # on the facet
            assert is_inside(link, x, side * (y - 0.4), z) and not is_inside(link, x, side * (y + 0.4), z), (side, t)
        x = R.wall_x[0] - R.facet_len - 1.0
        assert is_inside(link, x, side * (foot - 0.3), w.z0 + EDGE_CHAMFER + 0.3)
        assert is_inside(link, x, side * (foot - EDGE_CHAMFER - 0.3), w.z0 + 0.3)
        # the gussets' feet cut by the same facet near the wall, standing on it higher up
        assert not is_inside(link, R.wall_x[0] - 0.5, side * (apo - 0.5), w.z1 + 1.0)
        assert is_inside(link, R.wall_x[0] - 0.5, side * (apo - 0.5), w.z1 + 8.0)
    rr, rb = (R.flange_dia + R.flange_recess_add) / 2.0, R.cable_bore / 2.0
    assert not is_inside(link, x_face - 1.0, 0, 25 + rr - 1.0) and is_inside(link, x_face - 3.0, 0, 25 + rr - 1.0)   # the spigot recess, 2 deep
    assert not is_inside(link, xm, 0, 25) and not is_inside(link, xm, 0, 25 + rb - 1.0)               # the cable bore
    assert is_inside(link, xm, 0, 25 + rb + 1.5)
    # every screw goes in from the wrist side: its head's room behind the wall is open, over the web (the bottom one's
    # just over its top face, the web whole under it)
    head_r = R.screw.head_dia / 2.0
    for y, z in flange_bolt_points(DEFAULT):
        assert not is_inside(link, xm, y, z) and not is_inside(link, R.wall_x[0] + 0.1, y, z)
        for dy, dz in ((0.0, 0.0), (head_r, 0.0), (-head_r, 0.0), (0.0, head_r), (0.0, -head_r)):
            assert not is_inside(link, R.wall_x[0] - 1.0, y + dy, z + dz)
            assert not is_inside(link, R.wall_x[0] - R.screw.head_h - 1.0, y + dy, z + dz)
    xc = (DEFAULT.motor_x + DEFAULT.holes.pilot_dia / 2.0 + R.wall_x[0]) / 2.0     # between the pilot hole and the wall
    for z in (w.z0 + 0.3, zm, w.z1 - 0.3):
        assert is_inside(link, xc, 0, z) and is_inside(link, R.wall_x[0] - 1.0, 0, z)                     # no channel
    assert not is_inside(link, R.wall_x[0] - 1.0, 0, w.z1 + 0.3)
    # the round holes: the pilot's passes the Ø22 boss, the 31 mm square's M3 clearance; the web whole round them where
    # the slots were
    h, x = DEFAULT.holes, DEFAULT.motor_x
    assert not is_inside(link, x, 11.0, zm) and is_inside(link, x, 11.3, zm)
    for sx in (-h.bolt_half, h.bolt_half):
        for sy in (-h.bolt_half, h.bolt_half):
            assert not is_inside(link, x + sx, sy, zm) and is_inside(link, x + sx + h.bolt_dia / 2.0 + 0.3, sy, zm)
    assert is_inside(link, x - 14.0, 0, zm) and is_inside(link, x, 15.5 + 3.0, zm)
    assert is_inside(link, R.wall_x[0] - 4.0, 0, zm)
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
    board clear of j2_link, the pilot in its round hole: tests/test_mounts.py
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
