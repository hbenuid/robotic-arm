"""j1_link, parametric (lib/upper_arm/): the LEGACY configuration reproduces the SolidWorks part feature by feature
(the volume / bbox match is tests/test_reference_match.py's; here the features are probed by name so a regression
names what moved), and DEFAULT - what the part builds - changes exactly these: SHORTENING at the elbow end (every
elbow-end feature moves with the axis), no through slots, no cap sockets, the elbow block's clearance (the relief round
the elbow axis, the recess floor deeper), and the cycloidal drive's turning shell: the round root bolted to the shell by
the drive's housing bolts (their nuts in pockets from the underside) round the hole its held hub passes through
(ShellMountParams), the elbow motor's pad - the NEMA 17 holes on the motor's pattern - moved out along the link to the
stock elbow belt's centre distance from the elbow axis, where the second stage's seat was (gone). DEFAULT's own numbers
(volume, box) are locked here."""
import math
from dataclasses import replace

import pytest
from build123d import Location

from lib import mounts
from lib import params as PARAMS
from lib import placements as P
from lib import reference as R
from lib.belts import GT2_PULLEY_20T_TEETH, GT2_PULLEY_90T_TEETH, centre_distance
from lib.cycloidal import housing_bolt_points, stack_positions
from lib.cycloidal.params import DEFAULT_CONFIG
from lib.datum import to_location
from lib.motors import NEMA17_BOLT_SP
from lib.upper_arm import DEFAULT, LEGACY, hub_bolt_points, pad_holes, shell_bolt_points, socket_points
from lib.upper_arm.params import ELBOW_BELT, ELBOW_MOTOR_CENTRES, SHORTENING
from tests import built
from tests.helpers import is_inside


def _in_link(key: str, local=(0.0, 0.0, 0.0)) -> Location:
    """A point of occurrence `key` (in its own frame) in j1_link#1's frame."""
    return P.location("j1_link#1", "world").inverse() * P.location(key, "world") * Location(local)


def test_layout():
    assert len(hub_bolt_points(LEGACY)) == 4 and len(shell_bolt_points(LEGACY)) == 0
    assert len(shell_bolt_points(DEFAULT)) == DEFAULT_CONFIG.housing.bolt_count
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
    # the elbow motor off the shoulder axis (the drive's yoke holds it now): its pad the stock belt's centre distance
    # from the elbow, where the second stage's seat was - gone
    assert LEGACY.pad.x == 0.0 and DEFAULT.bearing is None and LEGACY.bearing is not None
    assert ELBOW_MOTOR_CENTRES == centre_distance(ELBOW_BELT, GT2_PULLEY_20T_TEETH, GT2_PULLEY_90T_TEETH) == pytest.approx(81.97, abs=0.01)
    assert DEFAULT.pad.x == pytest.approx(DEFAULT.slab.elbow_x - ELBOW_MOTOR_CENTRES, abs=1e-6)
    assert replace(DEFAULT.pad, x=0.0, holes=LEGACY.pad.holes) == LEGACY.pad
    assert PARAMS.J1_MOTOR_PAD_FACE_Y == DEFAULT.pad.face_y == -32.5 and PARAMS.ELBOW_BELT_LENGTH == ELBOW_BELT


def test_default_pad_holes_are_the_motors_pattern():
    """The elbow motor (lib/mounts.py nema17_40mm#2: face on the pad, shaft +N on the pad's axis) has its 4 bolts
    on the NEMA 17 square in its own frame (face z = 0): each one is a DEFAULT hole's axis (the holes about the pad's x)."""
    m = mounts.BY_KEY["nema17_40mm#2"]
    frame = to_location(m.frame)
    half = NEMA17_BOLT_SP / 2.0
    bolts = sorted((round(p.X, 6), round(p.Z, 6)) for p in (
        (frame * Location((sx * half, sy * half, 0.0))).position for sx in (1, -1) for sy in (1, -1)))
    holes = sorted((round(DEFAULT.pad.x + x, 6), round(z, 6)) for x, z, _ in pad_holes(DEFAULT))
    assert bolts == holes
    assert all((frame * Location((sx * half, sy * half, 0.0))).position.Y == pytest.approx(DEFAULT.pad.face_y)
               for sx in (1, -1) for sy in (1, -1))
    # LEGACY's holes were 0.38 off the axis and uneven - the reason DEFAULT moves them
    cx = sum(x for x, _, _ in pad_holes(LEGACY)) / 4.0
    cz = sum(z for _, z, _ in pad_holes(LEGACY)) / 4.0
    assert 0.35 < math.hypot(cx, cz) < 0.4


def test_default_shell_holes_are_the_drives_housing_bolts():
    """The drive's housing bolts (lib/cycloidal/layout.py housing_bolt_points), placed by placements.json, run along -Y
    through j1_link on DEFAULT's shell holes (the [REFERENCE] bolt angle re-derived), the shell's output face on the top
    face; their nuts (stack z_housing_nuts) seat on the pockets' floors; the held hub passes the hole 2 a side."""
    m, face_z = DEFAULT.shell, PARAMS.CYCLOIDAL_OUTPUT_FACE_Z
    holes = [(x, z) for x, z, _ in shell_bolt_points(DEFAULT)]
    nut_z = stack_positions(DEFAULT_CONFIG)["z_housing_nuts"]
    for x, y in housing_bolt_points(DEFAULT_CONFIG):
        head = _in_link("cycloidal_drive#1", (x, y, face_z)).position
        tip = _in_link("cycloidal_drive#1", (x, y, face_z + 10.0)).position
        assert abs(abs((tip - head).Y) - 10.0) < 1e-6                              # along the link's Y
        assert head.Y == pytest.approx(DEFAULT.slab.lip_top, abs=1e-5)            # the shell's face on the top face
        assert min(math.hypot(head.X - hx, head.Z - hz) for hx, hz in holes) < 1e-4
        nut = _in_link("cycloidal_drive#1", (x, y, nut_z)).position
        assert nut.Y == pytest.approx(DEFAULT.slab.y0 + m.nut_depth, abs=1e-5)    # the nut's inner face on the floor
    assert m.bolt_angle_deg == pytest.approx(DEFAULT.hub.bolt_angle_deg - DEFAULT_CONFIG.output_hub.arm_mount_angle_offset_deg, abs=1e-6)
    assert (m.bolt_circle_dia, m.bolt_count) == (PARAMS.CYCLOIDAL_SHELL_BOLT_CIRCLE, PARAMS.CYCLOIDAL_SHELL_BOLT_COUNT)
    assert (m.nut_af, m.nut_turn_deg) == (DEFAULT_CONFIG.housing.bolt_nut_pocket_af, DEFAULT_CONFIG.housing.bolt_nut_turn_deg)
    assert m.hole_dia - DEFAULT_CONFIG.output_hub.od == pytest.approx(4.1)
    assert m.root_r - m.bolt_circle_dia / 2.0 - m.nut_af / math.sqrt(3.0) > 3.0          # plastic past the pockets


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
    """DEFAULT's numbers (no reference holds them) and its features: the shortened elbow end, no sockets (the through
    slots' places are the pad's opening now), the elbow's relief and deeper recess; the round root on the shell - its hole, the shell's bolts and their
    nut pockets -; the pad moved out to its x, nothing of it left on the shoulder axis; the second stage's seat gone."""
    dx = SHORTENING
    assert link.is_valid and len(link.solids()) == 1
    assert R.solid_volume(link) == pytest.approx(308908.639, abs=0.5)
    bb, m = link.bounding_box(), DEFAULT.shell
    assert (bb.min.X, bb.min.Y, bb.min.Z) == pytest.approx((-m.root_r, DEFAULT.pad.face_y, -m.root_r), abs=1e-6)
    assert (bb.max.X, bb.max.Y, bb.max.Z) == pytest.approx((legacy.bounding_box().max.X + dx, DEFAULT.slab.lip_top, m.root_r), abs=1e-6)
    shoulder, elbow = socket_points(LEGACY)
    for x, z in shoulder:
        if math.hypot(x, z) > m.hole_dia / 2.0 + 1.0 and abs(x - DEFAULT.pad.x) > DEFAULT.hub.opening_half + 1.0:
            assert is_inside(link, x, -11.0, z)                                       # no sockets
    for x, z in elbow:
        assert is_inside(link, x + dx, -23.0, z)
    # the root and the hole
    y = (DEFAULT.slab.y0 + DEFAULT.slab.lip_top) / 2.0
    assert is_inside(link, -(m.root_r - 0.3), y, 0) and not is_inside(link, -(m.root_r + 0.3), y, 0)
    assert not is_inside(link, 0, y, m.hole_dia / 2.0 - 0.3) and is_inside(link, 0, y, m.hole_dia / 2.0 + 0.3)
    for x, z, a in shell_bolt_points(DEFAULT):
        assert not is_inside(link, x, DEFAULT.slab.lip_top - 0.3, z)                  # the bolt's hole, through ...
        u = (math.cos(math.radians(a)), math.sin(math.radians(a)))                    # ... its nut's pocket, a flat outward
        flat = m.nut_af / 2.0
        assert not is_inside(link, x + u[0] * (flat - 0.3), DEFAULT.slab.y0 + 0.5, z + u[1] * (flat - 0.3))
        assert is_inside(link, x + u[0] * (flat + 0.3), DEFAULT.slab.y0 + 0.5, z + u[1] * (flat + 0.3))
        assert is_inside(link, x + u[0] * (flat - 0.3), DEFAULT.slab.y0 + m.nut_depth + 0.3, z + u[1] * (flat - 0.3))
    # the pad at its x (its face, the pilot opening, the NEMA 17 holes), nothing of it left under the shoulder axis
    px = DEFAULT.pad.x
    assert is_inside(link, px + 20, -32, 20) and not is_inside(link, px + 20, -33, 20)
    assert not is_inside(link, px, -28, 0) and is_inside(link, px - 16, -28, 0)
    for x, z, _ in pad_holes(DEFAULT):
        assert not is_inside(link, px + x, -28, z)
    assert not is_inside(link, 20, -32, 20) and is_inside(legacy, 20, -32, 20)
    assert not is_inside(link, px, -5, 0)                                             # the plate's opening over it
    # the second stage's seat is gone (its boss under the plate, its web) - the pad's opening took its place
    b = LEGACY.bearing
    assert not is_inside(link, b.x + dx, -14, 15) and is_inside(legacy, b.x, -14, 15)
    # the elbow-end features where the shortening puts them: the stepped slot, the chamfer
    sl = DEFAULT.slots
    assert not is_inside(link, sl.stepped_x, -5, 0) and is_inside(link, sl.stepped_x + 4.0, -5, 0)
    assert is_inside(link, 149.5 + dx, -15.5, 30) and not is_inside(link, 149.5 + dx, -16.0, 30)
    # the elbow: the top face down to relief_y within relief_r of the axis, the lip still there beyond; the recess floor
    s, e = DEFAULT.slab, DEFAULT.elbow
    x_edge = s.elbow_x - e.relief_r
    assert not is_inside(link, x_edge + 1.0, e.relief_y + 0.5, 0) and is_inside(link, x_edge + 1.0, e.relief_y - 0.5, 0)
    assert is_inside(link, x_edge - 1.0, s.y1 + 1.0, 30) and is_inside(legacy, x_edge - dx + 1.0, s.y1 + 1.0, 30)   # beside the pad's opening
    r_floor = (e.recess_dia + e.bore_dia) / 4.0
    assert not is_inside(link, s.elbow_x, e.recess_y + 0.5, r_floor) and is_inside(link, s.elbow_x, e.recess_y - 0.5, r_floor)
