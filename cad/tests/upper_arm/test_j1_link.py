"""j1_link, parametric (lib/upper_arm/): the LEGACY configuration reproduces the SolidWorks part feature by feature
(the volume / bbox match is tests/test_reference_match.py's; here the features are probed by name so a regression
names what moved), and DEFAULT - what the part builds - changes exactly these: SHORTENING at the elbow end (every
elbow-end feature moves with the axis), no through slots, no cap sockets, the elbow block's clearance (the relief round
the elbow axis, the recess floor deeper), and the cycloidal drive's turning shell: the arm, a plain bar, rises straight
off the shell's middle (ArmParams), printed as one with the shell's body round the discs - out of the two windows it
fills, the others open - the plate's elbow end slid along Y onto the arm's outer face (arm_slide); the elbow motor -
the NEMA 17 holes on its pattern - moved out along the link to the stock elbow belt's centre distance from the elbow
axis, where the second stage's seat was (gone), and turned over onto the arm's +Y side: down a hole through the bar
onto a plate under its outer face, the pad gone. DEFAULT's own numbers (volume, box) are locked here."""
import math
from dataclasses import replace

import pytest
from build123d import Location

from lib import mounts
from lib import params as PARAMS
from lib import placements as P
from lib import reference as R
from lib.belts import GT2_PULLEY_20T_TEETH, GT2_PULLEY_90T_TEETH, centre_distance
from lib.cycloidal import arm_zone, housing_bolt_points, shell_ends, stack_positions
from lib.cycloidal.params import DEFAULT_CONFIG
from lib.datum import to_location
from lib.motors import NEMA17_BOLT_SP
from lib.upper_arm import DEFAULT, LEGACY, arm_slide, hub_bolt_points, pad_holes, socket_points
from lib.upper_arm.link import drive_frame
from lib.upper_arm.params import ELBOW_BELT, ELBOW_MOTOR_CENTRES, SHORTENING
from tests import built
from tests.helpers import is_inside

SLIDE = arm_slide(DEFAULT)   # 39.27


def _in_link(key: str, local=(0.0, 0.0, 0.0)) -> Location:
    """A point of occurrence `key` (in its own frame) in j1_link#1's frame."""
    return P.location("j1_link#1", "world").inverse() * P.location(key, "world") * Location(local)


def _drive(x: float, y: float, z: float) -> tuple:
    """A point of the drive's frame in j1_link's (ArmParams.drive_z_at_y0 / drive_x_deg)."""
    p = (drive_frame(DEFAULT) * Location((x, y, z))).position
    return (p.X, p.Y, p.Z)


def test_layout():
    assert len(hub_bolt_points(LEGACY)) == 4 and LEGACY.arm is None and arm_slide(LEGACY) == 0.0
    assert [len(s) for s in socket_points(LEGACY)] == [7, 3] and socket_points(DEFAULT) == ([], [])
    # SHORTENING at the elbow end: the axis and every elbow-end feature move with it; the shoulder end stays; the
    # through slots are gone
    dx = SHORTENING
    assert dx < 0.0 and DEFAULT.slab == replace(LEGACY.slab, elbow_x=LEGACY.slab.elbow_x + dx)
    assert DEFAULT.slots == replace(LEGACY.slots, through_x=(), stepped_x=LEGACY.slots.stepped_x + dx)
    assert LEGACY.elbow.relief_r == 0.0 and DEFAULT.elbow.relief_r > 0.0 and DEFAULT.elbow.recess_y < LEGACY.elbow.recess_y
    assert replace(DEFAULT.elbow, relief_r=LEGACY.elbow.relief_r, relief_y=LEGACY.elbow.relief_y,
                   recess_y=LEGACY.elbow.recess_y, chamfer_x=DEFAULT.elbow.chamfer_x - dx,
                   step_x=DEFAULT.elbow.step_x - dx) == LEGACY.elbow
    # the elbow motor off the shoulder axis (the drive's yoke holds it now): the stock belt's centre distance from the
    # elbow, where the second stage's seat was - gone; turned over onto the arm's +Y side, its face on its plate under
    # the arm's outer face (LEGACY's pad face stays in PadParams)
    assert LEGACY.pad.x == 0.0 and DEFAULT.bearing is None and LEGACY.bearing is not None
    assert ELBOW_MOTOR_CENTRES == centre_distance(ELBOW_BELT, GT2_PULLEY_20T_TEETH, GT2_PULLEY_90T_TEETH) == pytest.approx(81.97, abs=0.01)
    assert DEFAULT.pad.x == pytest.approx(DEFAULT.slab.elbow_x - ELBOW_MOTOR_CENTRES, abs=1e-6)
    assert replace(DEFAULT.pad, x=0.0, holes=LEGACY.pad.holes) == LEGACY.pad
    assert DEFAULT.pad.face_y == -32.5 and PARAMS.J1_MOTOR_PAD_FACE_Y == DEFAULT.arm.y_outer == 27.5
    assert PARAMS.J1_ARM_SLIDE == SLIDE and PARAMS.ELBOW_BELT_LENGTH == ELBOW_BELT


def test_the_arm_rises_off_the_drives_shell():
    """ArmParams: the drive's frame in this one is the capture's (placements.json cycloidal_drive#1 and j1_link#1, to
    1e-6); the arm spans the drive's arm_zone (its two plates' inner faces, the middle of the discs between) and the
    plate's underside slides onto its outer face; the fusion past the housing bolts' holes and inside the od; the
    drive's first pillar (HousingParams.bolt_start_deg, lib/cycloidal/params.py ARM_DEG) on the arm's centreline -
    the link's +X -, the windows either side of it the arm's root (ShellParams.arm_windows)."""
    a, h = DEFAULT.arm, DEFAULT_CONFIG.housing
    capture = P.location("j1_link#1", "world").inverse() * P.location("cycloidal_drive#1", "world")
    mine = drive_frame(DEFAULT)
    for v in ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)):
        assert tuple((capture * Location(v)).position) == pytest.approx(tuple((mine * Location(v)).position), abs=1e-6)
    z0, z1 = arm_zone(DEFAULT_CONFIG)
    assert (a.y_inner, a.y_outer) == (a.drive_z_at_y0 - z0, a.drive_z_at_y0 - z1) == (57.5, 27.5)
    assert SLIDE == pytest.approx(a.y_outer - DEFAULT.slab.y0) == pytest.approx(39.271993)
    assert h.od / 2.0 > a.fuse_r > (h.bolt_circle_dia + h.bolt_dia + DEFAULT_CONFIG.tolerances.bolt_clearance_add) / 2.0
    assert a.fuse_r > h.bore_dia / 2.0
    along = mine.inverse() * Location((1.0, 0.0, 0.0))
    origin = mine.inverse() * Location((0.0, 0.0, 0.0))
    d = along.position - origin.position
    assert math.degrees(math.atan2(d.Y, d.X)) == pytest.approx(h.bolt_start_deg, abs=1e-6) == pytest.approx(-a.drive_x_deg, abs=1e-6)
    assert DEFAULT_CONFIG.shell.arm_windows == 2 and 360.0 / h.bolt_count * DEFAULT_CONFIG.shell.arm_windows / 2.0 == 60.0


def test_default_motor_holes_are_the_motors_pattern():
    """The elbow motor (lib/mounts.py nema17_40mm#2: face on its plate, shaft -N on the pad's axis, the body +N) has
    its 4 bolts on the NEMA 17 square in its own frame (face z = 0): each one is a DEFAULT hole's axis (the holes about
    the pad's x), on the plate's face under the arm's outer face."""
    m = mounts.BY_KEY["nema17_40mm#2"]
    frame = to_location(m.frame)
    half = NEMA17_BOLT_SP / 2.0
    bolts = sorted((round(p.X, 6), round(p.Z, 6)) for p in (
        (frame * Location((sx * half, sy * half, 0.0))).position for sx in (1, -1) for sy in (1, -1)))
    holes = sorted((round(DEFAULT.pad.x + x, 6), round(z, 6)) for x, z, _ in pad_holes(DEFAULT))
    assert bolts == holes
    assert all((frame * Location((sx * half, sy * half, 0.0))).position.Y == pytest.approx(DEFAULT.arm.y_outer)
               for sx in (1, -1) for sy in (1, -1))
    shaft = (frame * Location((0.0, 0.0, 1.0))).position - frame.position
    assert tuple(shaft) == pytest.approx((0.0, -1.0, 0.0), abs=1e-9)                # shaft -N, through the plate
    # LEGACY's holes were 0.38 off the axis and uneven - the reason DEFAULT moves them
    cx = sum(x for x, _, _ in pad_holes(LEGACY)) / 4.0
    cz = sum(z for _, z, _ in pad_holes(LEGACY)) / 4.0
    assert 0.35 < math.hypot(cx, cz) < 0.4


def test_the_drives_housing_bolts_run_along_the_links_y():
    """The drive's housing bolts, placed by placements.json, run along -Y through j1_link's body: their seats at the
    drive's z through ArmParams.drive_z_at_y0."""
    nut_z, (_, end) = stack_positions(DEFAULT_CONFIG)["z_housing_nuts"], shell_ends(DEFAULT_CONFIG)
    for x, y in housing_bolt_points(DEFAULT_CONFIG):
        head = _in_link("cycloidal_drive#1", (x, y, 0.0)).position
        tip = _in_link("cycloidal_drive#1", (x, y, 10.0)).position
        assert abs(abs((tip - head).Y) - 10.0) < 1e-6                              # along the link's Y
        for z in (nut_z, end):
            assert _in_link("cycloidal_drive#1", (x, y, z)).position.Y == pytest.approx(DEFAULT.arm.drive_z_at_y0 - z, abs=1e-5)


@pytest.fixture(scope="module")
def legacy():
    return built.legacy("j1_link")


@pytest.fixture(scope="module")
def link():
    return built.part("j1_link")


@pytest.mark.slow
def test_legacy_features(legacy):
    leg = legacy
    assert leg.is_valid and len(leg.solids()) == 1
    assert is_inside(leg, 50, -5, 0) and is_inside(leg, 50, 1.0, 0)                  # the plate and its lip
    assert not is_inside(leg, 50, 1.0, 44.75) and is_inside(leg, 50, -1.0, 44.75)     # the lip is inset 0.5
    assert not is_inside(leg, 50, -12.5, 0) and is_inside(leg, 50, -11.5, 0)          # the shoulder half's underside
    assert is_inside(leg, 149.5, -15.5, 30) and not is_inside(leg, 149.5, -16.0, 30)  # the 45 degree chamfer
    assert is_inside(leg, 180, -23.5, 30) and not is_inside(leg, 150, -23.5, 30)      # the elbow half's step down
    assert not is_inside(leg, 0, -5, 0) and is_inside(leg, -22, -5, 0)                # the square opening
    for x, z in hub_bolt_points(LEGACY):
        assert not is_inside(leg, x, -5, z)                                           # the hub bolts
    assert is_inside(leg, 20, -32, 20) and not is_inside(leg, 20, -33, 20)            # the pad's face
    assert not is_inside(leg, 0, -28, 0) and is_inside(leg, -16, -28, 0)              # the pilot opening, the -X floor
    assert not is_inside(leg, 18, -28, 0)                                             # the +X window through the floor
    assert not is_inside(leg, 0, -20, 15) and is_inside(leg, 15, -20, 21)             # the cavity, a Z wall
    assert not is_inside(leg, 0, -20, 21) and not is_inside(leg, -22, -20, 0)         # the Z and -X windows
    assert is_inside(leg, -22, -20, 15)                                               # the -X wall beside its window
    for x, z, _ in pad_holes(LEGACY):
        assert not is_inside(leg, x, -28, z)                                          # the NEMA 17 holes
    assert is_inside(leg, 25.5, -12.5, 20) and not is_inside(leg, 25.5, -14.0, 20)    # the root flare
    assert is_inside(leg, -20.5, -9, 15) and not is_inside(leg, -20.5, -4, 15)        # the -X cove
    assert is_inside(leg, 15, -10.5, 20.5) and not is_inside(leg, 15, -9, 20.5)       # a Z cove
    assert not is_inside(leg, 128, -3, 10) and is_inside(leg, 128, -7, 6)             # x 128: the top seat, the web ...
    assert not is_inside(leg, 128, -7, 0)                                             # ... its hole ...
    assert is_inside(leg, 128, -14, 15) and not is_inside(leg, 128, -14, 21)          # ... the boss round the lower seat
    assert not is_inside(leg, 70.5, -5, 0) and not is_inside(leg, 100.5, -5, 25)      # the through slots ...
    assert is_inside(leg, 70.5, -5, 27)                                               # ... end at z +/- 26
    assert not is_inside(leg, 163, 0, 0) and not is_inside(leg, 160, -15, 0)          # the stepped slot and its counterbore
    assert is_inside(leg, 160, -5, 0)
    assert not is_inside(leg, 210, 0, 39) and is_inside(leg, 210, -6, 30)             # the elbow: recess ...
    assert not is_inside(leg, 210, -6, 20) and is_inside(leg, 210, -14, 20)           # ... bore, lip ...
    assert not is_inside(leg, 210, -20, 21)                                           # ... seat
    shoulder, elbow = socket_points(LEGACY)
    for x, z in shoulder:
        assert not is_inside(leg, x, -11.0, z) and is_inside(leg, x, -9.5, z)         # the cap's sockets
    for x, z in elbow:
        assert not is_inside(leg, x, -23.0, z) and is_inside(leg, x, -21.5, z)




@pytest.mark.slow
def test_default(link, legacy):
    """DEFAULT's numbers (no reference holds them) and its features: the arm off the shell - the bar, its faces and
    sides, the step and the relief near the elbow -, the shell's body inside, its windows under the bar solid and the
    others open; the plate's shortened elbow end slid onto the arm's outer face; the elbow motor's hole through the bar
    at the pad's x and its plate under it; no pad, no sockets, the second stage's seat gone."""
    dx, a = SHORTENING, DEFAULT.arm
    assert link.is_valid and len(link.solids()) == 1
    assert R.solid_volume(link) == pytest.approx(376960.613, abs=0.5)
    bb, hh = link.bounding_box(), DEFAULT_CONFIG.housing
    # across (Z) the body's pillars either side of +/-Z reach furthest: the windows on +/-Z are open
    assert (bb.min.X, bb.min.Y, bb.min.Z) == pytest.approx((-hh.od / 2.0, a.drive_z_at_y0 - shell_ends(DEFAULT_CONFIG)[1], -58.0117), abs=1e-4)
    assert (bb.max.X, bb.max.Y, bb.max.Z) == pytest.approx((legacy.bounding_box().max.X + dx, a.y_inner, 58.0117), abs=1e-4)
    # the bar: full width near the shell, its faces, its sides
    s, e = DEFAULT.slab, DEFAULT.elbow
    assert is_inside(link, 70, a.y_inner - 0.5, 30) and not is_inside(link, 70, a.y_inner + 0.5, 30)
    assert not is_inside(link, 70, a.y_outer - 0.5, 30)
    y = (a.y_inner + a.y_outer) / 2.0
    assert is_inside(link, 120, y, s.r - 0.3) and not is_inside(link, 120, y, s.r + 0.3)
    # the body's windows round the discs: the two either side of the arm's pillar solid, the others open (no collar)
    start = math.radians(hh.bolt_start_deg)
    r_win = (hh.bore_dia + hh.od) / 4.0
    for k, solid in ((1, True), (-1, True), (3, False), (-3, False), (5, False), (-5, False)):
        t = start + k * math.pi / hh.bolt_count
        assert is_inside(link, *_drive(r_win * math.cos(t), r_win * math.sin(t), 24.0)) == solid, f"window {k}"
    # ... stepped down over the elbow: the step (the roll drive's stator), then the relief
    cut_y = s.lip_top + SLIDE + a.swing_above_top
    assert not is_inside(link, 100, cut_y + 1.0, 30) and is_inside(link, 100, cut_y - 1.0, 30)
    step_x = s.elbow_x - a.swing_r
    assert is_inside(link, step_x - 1.0, cut_y + 1.0, 30) and not is_inside(link, step_x + 1.0, cut_y + 1.0, 30)
    x_edge = s.elbow_x - e.relief_r
    assert not is_inside(link, x_edge + 1.0, e.relief_y + SLIDE + 0.5, 0) and is_inside(link, x_edge + 1.0, e.relief_y + SLIDE - 0.5, 0)
    assert is_inside(link, x_edge - 1.0, e.relief_y + SLIDE + 0.5, 30)              # the arm, outside the relief
    # the shell's body inside, at the drive's places: the bore round the discs, the hub-end seat and lip, the bolts'
    # holes through the collar, the nuts' pockets
    sh, hh, (z_0, z_1) = DEFAULT_CONFIG.shell, DEFAULT_CONFIG.housing, arm_zone(DEFAULT_CONFIG)
    mid = (z_0 + z_1) / 2.0
    assert not is_inside(link, *_drive(hh.bore_dia / 2.0 - 0.3, 0.0, mid)) and is_inside(link, *_drive(hh.bore_dia / 2.0 + 0.3, 0.0, mid))
    seat = DEFAULT_CONFIG.stack_up.z_output_bearings + 5.0
    assert not is_inside(link, *_drive(hh.output_bearing_seat_dia / 2.0 - 0.1, 0.0, seat))
    assert is_inside(link, *_drive(hh.lip_bore_dia / 2.0 + 0.3, 0.0, shell_ends(DEFAULT_CONFIG)[1] - sh.end_lip / 2.0))
    nut_z = stack_positions(DEFAULT_CONFIG)["z_housing_nuts"]
    for x, y_ in housing_bolt_points(DEFAULT_CONFIG):
        assert not is_inside(link, *_drive(x, y_, mid)) and not is_inside(link, *_drive(x, y_, nut_z + 1.0))
    # the plate's elbow end, slid: the elbow bearing stack, the stepped slot, the chamfer
    r_floor = (e.recess_dia + e.bore_dia) / 4.0
    assert not is_inside(link, s.elbow_x, e.recess_y + SLIDE + 0.5, r_floor) and is_inside(link, s.elbow_x, e.recess_y + SLIDE - 0.5, r_floor)
    sl = DEFAULT.slots
    assert not is_inside(link, sl.stepped_x, -5 + SLIDE, 0) and is_inside(link, sl.stepped_x + 4.0, -5 + SLIDE, 0)
    assert is_inside(link, 149.5 + dx, -15.5 + SLIDE, 30) and not is_inside(link, 149.5 + dx, -16.0 + SLIDE, 30)
    for x, z in socket_points(LEGACY)[1]:
        assert is_inside(link, x + dx, -23.0 + SLIDE, z)                              # no sockets
    # the elbow motor at the pad's x: its hole through the bar, the bar beside it; its plate under the arm's outer face
    # (the motor's face on its top, the pilot's hole, the NEMA 17 holes); the pad's tube gone
    px, hole = DEFAULT.pad.x, a.motor_hole_half
    assert not is_inside(link, px, y, 0) and not is_inside(link, px + hole - 0.3, y, hole - 0.3)
    assert is_inside(link, px, y, hole + 0.3) and is_inside(link, px - hole - 0.3, y, 0)
    assert is_inside(link, px + 20, a.y_outer - 0.1, 20) and not is_inside(link, px + 20, a.y_outer + 0.1, 20)
    assert not is_inside(link, px + 20, a.y_outer - a.motor_plate_t - 0.1, 20)
    assert is_inside(link, px + a.motor_plate_half - 0.3, a.y_outer - 1.0, 0) and not is_inside(link, px + 20, a.y_outer - a.motor_plate_t - 1.0, 0)
    assert not is_inside(link, px, a.y_outer - 1.0, 0) and not is_inside(link, px + a.motor_pilot_dia / 2.0 - 0.3, a.y_outer - 1.0, 0)
    for x, z, _ in pad_holes(DEFAULT):
        assert not is_inside(link, px + x, a.y_outer - 1.0, z)
    assert not is_inside(link, px + 20, -28 + SLIDE, 20) and is_inside(legacy, 20, -28, 20)   # the pad's wall, LEGACY's only
